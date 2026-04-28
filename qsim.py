"""
Q-learning simulator for the 4-sensor obstacle-avoidance robot.

Trains a Q-table in a simulated arena, then writes q_table.h that can be
pasted into Q_learning.ino as the starting Q-table.

Run:    python qsim.py
Output: q_table.h
"""

import math
import random

# ----- Must match Q_learning.ino -----
NUM_STATES  = 16   # 2^4 sensors: F, L, R, B
NUM_ACTIONS = 4    # 0:Fwd, 1:Bwd, 2:Left, 3:Right
NEAR_CM     = 15   # threshold for "near" bit
BIT_F, BIT_L, BIT_R, BIT_B = 0, 1, 2, 3

# ----- Q-learning hyperparams -----
ALPHA          = 0.1
GAMMA          = 0.9
EPS_START      = 0.9
EPS_MIN        = 0.10
EPS_DECAY      = 0.9995
NUM_EPISODES   = 1000     # more training (was 500)
STEPS_PER_EP   = 200
STUCK_LIMIT    = 5
STUCK_PENALTY  = 50
NO_PROGRESS_LIMIT_STEPS = 10   # if we haven't moved >NO_PROGRESS_DIST_CM in this many steps -> oscillating
NO_PROGRESS_DIST_CM     = 8

# ----- Robot/arena model (rough; tune to your real bot) -----
ARENA_W_CM       = 200    # 2 m x 2 m bounded box
ARENA_H_CM       = 200
FORWARD_STEP_CM  = 10     # how far one Fwd/Bwd action moves
TURN_DEG         = 30     # how far one Left/Right action rotates
SENSOR_NOISE_CM  = 2      # gaussian noise on readings (sim-to-real help)
MAX_SENSOR_CM    = 200

# Sensor angles in degrees, relative to robot heading.
# The "Left" sensor on the real bot is mounted at the front-left diagonal,
# and "Right" at the front-right diagonal.
SENSOR_F_DEG = 0
SENSOR_L_DEG = 45     # front-left diagonal
SENSOR_R_DEG = -45    # front-right diagonal
SENSOR_B_DEG = 180

# Axis-aligned rectangular obstacles inside the arena: (x, y, w, h) in cm.
OBSTACLES = [
    (60, 60, 30, 30),     # small box left-of-center
    (130, 110, 25, 50),   # tall rectangle on the right
    (40, 150, 60, 20),    # wide rectangle near the top-left
]


def _point_in_obstacle(x, y):
    for (ox, oy, ow, oh) in OBSTACLES:
        if ox <= x <= ox + ow and oy <= y <= oy + oh:
            return True
    return False


def _ray_aabb_t(px, py, dx, dy, xmin, ymin, xmax, ymax):
    """Ray-vs-AABB. Returns t of nearest entry (>0), or None if no hit."""
    t_near = -float("inf")
    t_far  =  float("inf")
    for p, d, lo, hi in ((px, dx, xmin, xmax), (py, dy, ymin, ymax)):
        if abs(d) < 1e-9:
            if p < lo or p > hi:
                return None
        else:
            t1 = (lo - p) / d
            t2 = (hi - p) / d
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > t_near: t_near = t1
            if t2 < t_far:  t_far = t2
            if t_near > t_far:
                return None
    return t_near if t_near > 0 else None


class World:
    def __init__(self):
        self.reset()

    def reset(self):
        for _ in range(100):
            self.x = random.uniform(20, ARENA_W_CM - 20)
            self.y = random.uniform(20, ARENA_H_CM - 20)
            if not _point_in_obstacle(self.x, self.y):
                break
        self.theta = random.uniform(0, 2 * math.pi)

    def _ray(self, angle_offset_deg):
        """Cast a ray from robot. Returns cm to nearest wall or obstacle."""
        a = self.theta + math.radians(angle_offset_deg)
        dx, dy = math.cos(a), math.sin(a)
        ts = []
        # Walls
        if dx > 1e-9:  ts.append((ARENA_W_CM - self.x) / dx)
        if dx < -1e-9: ts.append((0 - self.x) / dx)
        if dy > 1e-9:  ts.append((ARENA_H_CM - self.y) / dy)
        if dy < -1e-9: ts.append((0 - self.y) / dy)
        # Obstacles
        for (ox, oy, ow, oh) in OBSTACLES:
            t = _ray_aabb_t(self.x, self.y, dx, dy, ox, oy, ox + ow, oy + oh)
            if t is not None:
                ts.append(t)
        ts = [t for t in ts if t > 0]
        d = min(ts) if ts else MAX_SENSOR_CM
        d += random.gauss(0, SENSOR_NOISE_CM)
        return max(0.0, min(d, MAX_SENSOR_CM))

    def get_state(self):
        dF = self._ray(SENSOR_F_DEG)
        dL = self._ray(SENSOR_L_DEG)
        dR = self._ray(SENSOR_R_DEG)
        dB = self._ray(SENSOR_B_DEG)
        s = 0
        if 0 < dF < NEAR_CM: s |= (1 << BIT_F)
        if 0 < dL < NEAR_CM: s |= (1 << BIT_L)
        if 0 < dR < NEAR_CM: s |= (1 << BIT_R)
        if 0 < dB < NEAR_CM: s |= (1 << BIT_B)
        return s

    def step(self, action):
        """Apply action. Returns True if the move would have collided."""
        nx, ny = self.x, self.y
        if action == 0:
            nx += FORWARD_STEP_CM * math.cos(self.theta)
            ny += FORWARD_STEP_CM * math.sin(self.theta)
        elif action == 1:
            nx -= FORWARD_STEP_CM * math.cos(self.theta)
            ny -= FORWARD_STEP_CM * math.sin(self.theta)
        elif action == 2:
            self.theta += math.radians(TURN_DEG)
            return False
        elif action == 3:
            self.theta -= math.radians(TURN_DEG)
            return False

        hit_wall = (nx <= 0 or nx >= ARENA_W_CM or
                    ny <= 0 or ny >= ARENA_H_CM)
        hit_obs  = _point_in_obstacle(nx, ny)
        crashed = hit_wall or hit_obs
        if not crashed:
            self.x, self.y = nx, ny
        return crashed


def reward_fn(action, next_state, crashed):
    """Mirrors calculate_reward() in Q_learning.ino, plus crash term."""
    front_near = next_state & (1 << BIT_F)
    back_near  = next_state & (1 << BIT_B)
    if crashed:                         return -100   # actual collision
    if action == 0 and front_near:      return -5     # mild warning (was -10)
    if action == 1 and back_near:       return -5     # mild warning (was -10)
    if action == 0 and not front_near:  return 20     # reward forward motion
    return -2                                          # turning


def argmax(row):
    return max(range(len(row)), key=lambda i: row[i])


def train():
    Q = [[0.0] * NUM_ACTIONS for _ in range(NUM_STATES)]
    eps = EPS_START
    world = World()

    for ep in range(NUM_EPISODES):
        world.reset()
        last_state = -1
        same_count = 0
        ep_reward = 0.0

        # Position history for oscillation detection
        anchor_x, anchor_y = world.x, world.y
        anchor_step = 0

        for step in range(STEPS_PER_EP):
            s = world.get_state()
            if random.random() < eps:
                a = random.randint(0, NUM_ACTIONS - 1)
            else:
                a = argmax(Q[s])

            crashed = world.step(a)
            s2 = world.get_state()
            r = reward_fn(a, s2, crashed)

            # Same-state stuck detection
            if s2 == last_state: same_count += 1
            else:                same_count = 0
            last_state = s2
            if same_count >= STUCK_LIMIT: r -= STUCK_PENALTY

            # Position-based oscillation detection (catches FWD/BWD loops)
            if step - anchor_step >= NO_PROGRESS_LIMIT_STEPS:
                dx = world.x - anchor_x
                dy = world.y - anchor_y
                if (dx * dx + dy * dy) ** 0.5 < NO_PROGRESS_DIST_CM:
                    r -= STUCK_PENALTY
                anchor_x, anchor_y = world.x, world.y
                anchor_step = step

            best_next = max(Q[s2])
            Q[s][a] += ALPHA * (r + GAMMA * best_next - Q[s][a])

            eps = max(EPS_MIN, eps * EPS_DECAY)
            ep_reward += r

        if ep % 50 == 0:
            print(f"ep {ep:4d}  eps={eps:.3f}  reward={ep_reward:7.1f}")

    return Q


def export_header(Q, path="q_table.h"):
    rows = []
    rows.append("// Auto-generated by qsim.py — pre-trained Q-table.")
    rows.append("// Include this from Q_learning.ino and copy into q_table on boot.")
    rows.append("#pragma once")
    rows.append("")
    rows.append(f"const float Q_TABLE_INIT[{NUM_STATES}][{NUM_ACTIONS}] = {{")
    for s in range(NUM_STATES):
        cells = ", ".join(f"{Q[s][a]:9.3f}f" for a in range(NUM_ACTIONS))
        rows.append(f"  {{ {cells} }},  // s={s:2d}  FLRB={s:04b}")
    rows.append("};")
    with open(path, "w") as f:
        f.write("\n".join(rows) + "\n")
    print(f"\nWrote {path}")


def show_policy(Q):
    names = ["FWD", "BWD", "LFT", "RGT"]
    print("\nLearned policy (best action per state):")
    print(f"{'state':>5}  FLRB    best   Q-values")
    for s in range(NUM_STATES):
        best = argmax(Q[s])
        qs = "  ".join(f"{q:7.2f}" for q in Q[s])
        print(f"{s:>5}  {s:04b}    {names[best]}    {qs}")


if __name__ == "__main__":
    random.seed(0)
    Q = train()
    show_policy(Q)
    export_header(Q)
