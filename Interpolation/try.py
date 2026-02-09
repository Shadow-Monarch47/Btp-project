import pygame
import numpy as np
from multiprocessing import Process, Queue
from collections import deque
import time

# ============================================================
# CONFIG
# ============================================================
GRID_SIZE = 55
CELL_SIZE = 15
WINDOW_SIZE = GRID_SIZE * CELL_SIZE

FPS = 60
MOVE_DURATION = 0.6          # seconds per grid move
LOOKAHEAD = 3                # buffered future waypoints

# ============================================================
# ROBOTS (YOUR DEFINITION, UNCHANGED LOGIC)
# ============================================================
robots = {
    "R1": {
        "path": [(1, 1), (1, 2), (1, 3), (1, 4),
                 (2, 4), (3, 4), (4, 4),
                 (4, 3), (4, 2), (4, 1),
                 (3, 1), (2, 1), (1, 1)],
        "color": (70, 140, 255),
    },
    "R2": {
        "path": [(3, 1), (3, 2), (3, 3), (3, 4)],
        "color": (255, 120, 70),
    }
}

# ============================================================
# PLANNER PROCESS (ONE PER ROBOT)
# ============================================================
def planner_process(path, queue):
    """
    Pushes future waypoints into a queue (lookahead).
    """
    for pos in path[1:]:
        time.sleep(0.01)          # simulate heavy computation
        queue.put(np.array(pos, dtype=float))

    queue.put(None)               # signal end of path


# ============================================================
# MAIN PROGRAM
# ============================================================
def main():
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    pygame.display.set_caption("Two Robots – Smooth Motion (No Wobble)")
    clock = pygame.time.Clock()

    # --------------------------------------------------------
    # INITIALIZE PER-ROBOT STATE
    # --------------------------------------------------------
    for r in robots.values():
        start = np.array(r["path"][0], dtype=float)
        r.update({
            "current": start.copy(),
            "start": start.copy(),
            "target": None,
            "future_targets": deque(),
            "elapsed": 0.0,
            "reached_goal": False,
            "queue": Queue()
        })

        p = Process(
            target=planner_process,
            args=(r["path"], r["queue"]),
            daemon=True
        )
        r["planner"] = p
        p.start()

    # --------------------------------------------------------
    # DRAW HELPERS
    # --------------------------------------------------------
    def draw_grid():
        for i in range(GRID_SIZE + 1):
            pygame.draw.line(
                screen, (200, 200, 200),
                (i * CELL_SIZE, 0),
                (i * CELL_SIZE, WINDOW_SIZE)
            )
            pygame.draw.line(
                screen, (200, 200, 200),
                (0, i * CELL_SIZE),
                (WINDOW_SIZE, i * CELL_SIZE)
            )

    def draw_robots():
        for r in robots.values():
            x, y = r["current"]
            screen_x = x * CELL_SIZE
            screen_y = (GRID_SIZE - 1 - y) * CELL_SIZE

            rect = pygame.Rect(
                screen_x + CELL_SIZE * 0.3,
                screen_y + CELL_SIZE * 0.3,
                CELL_SIZE * 0.4,
                CELL_SIZE * 0.4
            )
            pygame.draw.rect(screen, r["color"], rect)

    # --------------------------------------------------------
    # MAIN LOOP
    # --------------------------------------------------------
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # ----------------------------------------------------
        # UPDATE EACH ROBOT INDEPENDENTLY
        # ----------------------------------------------------
        for r in robots.values():

            # Fill lookahead buffer (non-blocking)
            while (
                not r["queue"].empty()
                and len(r["future_targets"]) < LOOKAHEAD
            ):
                item = r["queue"].get()
                if item is None:
                    r["reached_goal"] = True
                    break
                r["future_targets"].append(item)

            # Pick initial target
            if r["target"] is None and r["future_targets"]:
                r["target"] = r["future_targets"].popleft()
                r["start"] = r["current"].copy()
                r["elapsed"] = 0.0

            # Interpolate
            if r["target"] is not None:
                r["elapsed"] += dt
                alpha = min(r["elapsed"] / MOVE_DURATION, 1.0)

                r["current"][:] = (
                    (1 - alpha) * r["start"] +
                    alpha * r["target"]
                )

                # Seamless transition
                if alpha >= 1.0:
                    r["start"] = r["target"].copy()
                    r["target"] = (
                        r["future_targets"].popleft()
                        if r["future_targets"] else None
                    )
                    r["elapsed"] = 0.0

        # ----------------------------------------------------
        # RENDER
        # ----------------------------------------------------
        screen.fill((255, 255, 255))
        draw_grid()
        draw_robots()
        pygame.display.flip()

    # --------------------------------------------------------
    # CLEANUP
    # --------------------------------------------------------
    for r in robots.values():
        r["planner"].terminate()

    pygame.quit()


# ============================================================
# ENTRY POINT (REQUIRED FOR WINDOWS)
# ============================================================
if __name__ == "__main__":
    main()
