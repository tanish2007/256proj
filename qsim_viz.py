"""
Pygame visualization of the trained Q-learning policy.

Trains a Q-table headlessly (using qsim.py), then opens a window where you
can watch the trained robot drive around the simulated arena.

Run:    python qsim_viz.py
Needs:  pip install pygame

Controls:
    R       reset the robot to a random pose
    SPACE   pause / unpause
    T       toggle "training mode" (keep learning + exploring)
    ESC     quit
"""

import math
import random
import sys

import pygame

from qsim import (
    World,
    ARENA_W_CM, ARENA_H_CM, NEAR_CM, NUM_ACTIONS,
    ALPHA, GAMMA,
    SENSOR_F_DEG, SENSOR_L_DEG, SENSOR_R_DEG, SENSOR_B_DEG,
    OBSTACLES,
    train, argmax, reward_fn,
)

# --- Display config ---
SCALE       = 3                      # pixels per cm
ARENA_PX_W  = ARENA_W_CM * SCALE     # 600
ARENA_PX_H  = ARENA_H_CM * SCALE     # 600
PANEL_W     = 240
WIN_W       = ARENA_PX_W + PANEL_W
WIN_H       = ARENA_PX_H
FPS         = 8                      # slow enough to watch
ACTION_NAMES = ["FWD", "BWD", "LEFT", "RIGHT"]


def world_to_screen(x, y):
    """World (cm) -> screen (px). Flip y so +y is up on screen."""
    return int(x * SCALE), int(WIN_H - y * SCALE)


def _ray_aabb_t(px, py, dx, dy, xmin, ymin, xmax, ymax):
    t_near, t_far = -float("inf"), float("inf")
    for p, d, lo, hi in ((px, dx, xmin, xmax), (py, dy, ymin, ymax)):
        if abs(d) < 1e-9:
            if p < lo or p > hi:
                return None
        else:
            t1 = (lo - p) / d
            t2 = (hi - p) / d
            if t1 > t2: t1, t2 = t2, t1
            if t1 > t_near: t_near = t1
            if t2 < t_far:  t_far = t2
            if t_near > t_far:
                return None
    return t_near if t_near > 0 else None


def cast_ray_clean(world, angle_offset_deg):
    """Same ray cast as World._ray but without noise — for drawing."""
    a = world.theta + math.radians(angle_offset_deg)
    dx, dy = math.cos(a), math.sin(a)
    ts = []
    if dx > 1e-9:  ts.append((ARENA_W_CM - world.x) / dx)
    if dx < -1e-9: ts.append((0 - world.x) / dx)
    if dy > 1e-9:  ts.append((ARENA_H_CM - world.y) / dy)
    if dy < -1e-9: ts.append((0 - world.y) / dy)
    for (ox, oy, ow, oh) in OBSTACLES:
        t = _ray_aabb_t(world.x, world.y, dx, dy, ox, oy, ox + ow, oy + oh)
        if t is not None:
            ts.append(t)
    ts = [t for t in ts if t > 0]
    return min(ts) if ts else 200.0


def draw_arena(screen):
    screen.fill((25, 25, 30))
    pygame.draw.rect(screen, (180, 180, 200),
                     (0, 0, ARENA_PX_W, ARENA_PX_H), 3)
    for (ox, oy, ow, oh) in OBSTACLES:
        x_px, y_top_px = world_to_screen(ox, oy + oh)
        pygame.draw.rect(screen, (140, 90, 90),
                         (x_px, y_top_px, int(ow * SCALE), int(oh * SCALE)))


def draw_sensors(screen, world):
    cx, cy = world_to_screen(world.x, world.y)
    for offset_deg in (SENSOR_F_DEG, SENSOR_L_DEG, SENSOR_R_DEG, SENSOR_B_DEG):
        d = cast_ray_clean(world, offset_deg)
        a = world.theta + math.radians(offset_deg)
        ex, ey = world_to_screen(world.x + d * math.cos(a),
                                 world.y + d * math.sin(a))
        color = (255, 80, 80) if d < NEAR_CM else (90, 200, 110)
        pygame.draw.line(screen, color, (cx, cy), (ex, ey), 2)


def draw_robot(screen, world):
    cx, cy = world_to_screen(world.x, world.y)
    size = 14
    a = world.theta
    tip   = (cx + int(size * math.cos(a)),       cy - int(size * math.sin(a)))
    left  = (cx + int(size * 0.7 * math.cos(a + 2.5)),
             cy - int(size * 0.7 * math.sin(a + 2.5)))
    right = (cx + int(size * 0.7 * math.cos(a - 2.5)),
             cy - int(size * 0.7 * math.sin(a - 2.5)))
    pygame.draw.polygon(screen, (100, 180, 255), [tip, left, right])
    pygame.draw.circle(screen, (255, 255, 255), (cx, cy), 3)


def draw_panel(screen, font, lines):
    px = ARENA_PX_W
    pygame.draw.rect(screen, (40, 40, 50), (px, 0, PANEL_W, WIN_H))
    y = 16
    for line in lines:
        text = font.render(line, True, (230, 230, 240))
        screen.blit(text, (px + 14, y))
        y += 22


def main():
    print("Training Q-table headlessly (seed=0, so this matches q_table.h)...")
    Q = train(seed=0)
    print("Done. Opening visualization.")

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Q-learning robot — visualization")
    font = pygame.font.SysFont("consolas", 16)
    clock = pygame.time.Clock()

    world = World()
    paused = False
    training_mode = False
    epsilon = 0.2  # used only if training_mode is on

    step = 0
    total_reward = 0.0
    last_state = 0
    last_action = 0
    last_reward = 0.0

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    world.reset()
                    step = 0
                    total_reward = 0.0
                elif event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_t:
                    training_mode = not training_mode

        if not paused:
            s = world.get_state()
            if training_mode and random.random() < epsilon:
                a = random.randint(0, NUM_ACTIONS - 1)
            else:
                a = argmax(Q[s])

            crashed = world.step(a)
            s2 = world.get_state()
            r = reward_fn(a, s2, crashed)

            if training_mode:
                best_next = max(Q[s2])
                Q[s][a] += ALPHA * (r + GAMMA * best_next - Q[s][a])

            last_state, last_action, last_reward = s, a, r
            step += 1
            total_reward += r
            if crashed:
                world.reset()

        draw_arena(screen)
        draw_sensors(screen, world)
        draw_robot(screen, world)

        mode = "TRAINING" if training_mode else "GREEDY"
        bF = (last_state >> 0) & 1
        bL = (last_state >> 1) & 1
        bR = (last_state >> 2) & 1
        bB = (last_state >> 3) & 1
        info = [
            f"MODE:    {mode}",
            "",
            f"step:    {step}",
            f"state:   {last_state:>2d}",
            f"  F={bF}  L={bL}  R={bR}  B={bB}",
            f"action:  {ACTION_NAMES[last_action]}",
            f"reward:  {last_reward:+.0f}",
            f"total:   {total_reward:+.0f}",
            "",
            "Q for current state:",
        ]
        for i, name in enumerate(ACTION_NAMES):
            mark = "<" if i == last_action else " "
            info.append(f"  {name:5s} {Q[last_state][i]:+7.2f} {mark}")
        info += [
            "",
            "[R] reset",
            "[SPACE] pause",
            "[T] training mode",
            "[ESC] quit",
        ]
        draw_panel(screen, font, info)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


if __name__ == "__main__":
    main()
