import matplotlib.pyplot as plt
import matplotlib.animation as animation
import itertools
import numpy as np

# ---------------- CONFIG ----------------
GRID_SIZE = 10
FPS = 120
STEPS_PER_MOVE = FPS  # interpolate over 1 second
INTERVAL = int(1000 / FPS)

# ---------------- STATE ----------------
robots = {
    "R1": {
        "current": np.array([1.0, 1.0]),
        "start":   np.array([1.0, 1.0]),
        "target":  np.array([1.0, 1.0]),
    },
    "R2": {
        "current": np.array([3.0, 1.0]),
        "start":   np.array([3.0, 1.0]),
        "target":  np.array([3.0, 1.0]),
    },
}


interp_counter = 0  # global interpolation progress

# ---------------- FIGURE ----------------
fig, ax = plt.subplots()
ax.set_xlim(0, GRID_SIZE)
ax.set_ylim(0, GRID_SIZE)
ax.set_xticks(range(GRID_SIZE))
ax.set_yticks(range(GRID_SIZE))
ax.grid(True)

patches = {}
for rid, r in robots.items():
    rect = plt.Rectangle(r["current"], 0.8, 0.8)
    ax.add_patch(rect)
    patches[rid] = rect


# ============================================================
# 1. PLANNING FUNCTION (DISCRETE, LOGIC-ONLY)
# ============================================================
def compute_next_positions():
    """
    Computes next grid positions ONCE per logical step.
    Returns artists (patches).
    """
    global interp_counter

    # Example logic: move all robots UP by 1 cell
    for r in robots.values():
        r["start"] = r["current"].copy()
        r["target"] = r["current"] + np.array([1, 2])

    interp_counter = 0  # reset interpolation
    return patches.values()


# ============================================================
# 2. INTERPOLATION FUNCTION (CONTINUOUS, VISUAL)
# ============================================================
def interpolate_positions():
    """
    Interpolates robot positions at 30 FPS.
    """
    global interp_counter

    alpha = interp_counter / STEPS_PER_MOVE
    alpha = min(alpha, 1.0)

    for rid, r in robots.items():
        pos = (1 - alpha) * r["start"] + alpha * r["target"]
        r["current"] = pos
        patches[rid].set_xy(pos)

    interp_counter += 1
    return patches.values()


# ============================================================
# 3. FUNCANIMATION UPDATE (ORCHESTRATOR)
# ============================================================
def update(frame):
    """
    This function NEVER blocks.
    Both planning and interpolation run without stopping animation.
    """

    # Every STEPS_PER_MOVE frames, compute new targets
    if interp_counter >= STEPS_PER_MOVE:
        compute_next_positions()

    # Always interpolate
    return interpolate_positions()


# ---------------- INIT FUNC ----------------
def init():
    for rid, r in robots.items():
        patches[rid].set_xy(r["current"])
    return patches.values()


# ---------------- ANIMATION ----------------
ani = animation.FuncAnimation(
    fig,
    update,
    init_func=init,
    frames=itertools.count(),     # infinite frames
    interval=INTERVAL,
    blit=True,
    cache_frame_data=False
)

plt.show()
