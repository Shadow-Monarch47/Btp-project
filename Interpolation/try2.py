import pygame
import numpy as np

# ================= CONFIG =================
GRID_SIZE = 55
CELL_SIZE = 18
WINDOW_SIZE = GRID_SIZE * CELL_SIZE

FPS = 60
MOVE_DURATION = 0.6  # seconds per grid move

# ================= ROBOTS =================

obstacles = {(10, 10), (10, 8)}

robots = {
    "R1": {
        "path": [
            (1, 1), (1, 2), (1, 3), (1, 4),
            (2, 4), (3, 4), (4, 4),
            (4, 3), (4, 2), (4, 1),
            (3, 1), (2, 1), (1, 1)
        ],
        "color": (70, 140, 255),
    },
    "R2": {
        "path": [(3, 1), (3, 2), (3, 3), (3, 4)],
        "color": (255, 120, 70),
    }
}

# ================= INIT =================
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Simple Smooth Robot Animation with Trails")
clock = pygame.time.Clock()

# ================= INIT ROBOT STATE =================
for r in robots.values():
    r["index"] = 0
    r["start"] = np.array(r["path"][0], float)
    r["current"] = r["start"].copy()
    r["target"] = np.array(r["path"][1], float)
    r["elapsed"] = 0.0
    r["trail"] = []        # ⭐ NEW: store trail points

# ================= DRAW =================
def draw_grid():
    for i in range(GRID_SIZE + 1):
        pygame.draw.line(
            screen, (220, 220, 220),
            (i * CELL_SIZE, 0),
            (i * CELL_SIZE, WINDOW_SIZE)
        )
        pygame.draw.line(
            screen, (220, 220, 220),
            (0, i * CELL_SIZE),
            (WINDOW_SIZE, i * CELL_SIZE)
        )

        pygame.draw.rect(
            screen, (180, 180, 180),
            (0, i * CELL_SIZE, WINDOW_SIZE, 1)
        )
        pygame.draw.rect(
            screen, (180, 180, 180),
            (i * CELL_SIZE, 0, 1, WINDOW_SIZE)
        )


def draw_trails():
    for r in robots.values():
        for x, y in r["trail"]:
            screen_x = x * CELL_SIZE + CELL_SIZE // 2
            screen_y = (GRID_SIZE - 1 - y) * CELL_SIZE + CELL_SIZE // 2
            pygame.draw.circle(screen, r["color"], (screen_x, screen_y), 1)

def draw_obstacles():
    for (x, y) in obstacles:
        screen_x = x * CELL_SIZE
        screen_y = (GRID_SIZE - 1 - y) * CELL_SIZE

        rect = pygame.Rect(
            screen_x,
            screen_y,
            CELL_SIZE,
            CELL_SIZE
        )
        pygame.draw.rect(screen, (60, 60, 60), rect)  # dark gray


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

# ================= MAIN LOOP =================
running = True
while running:
    dt = clock.tick(FPS) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    for r in robots.values():
        r["elapsed"] += dt
        alpha = min(r["elapsed"] / MOVE_DURATION, 1.0)

        r["current"] = (
            (1 - alpha) * r["start"] +
            alpha * r["target"]
        )

        # ⭐ ADD TO TRAIL (smooth trail)
        r["trail"].append(tuple(r["current"]))

        # reached target → go to next
        if alpha >= 1.0:
            r["index"] += 1
            if r["index"] < len(r["path"]) - 1:
                r["start"] = r["target"]
                r["target"] = np.array(r["path"][r["index"] + 1], float)
                r["elapsed"] = 0.0

    screen.fill((255, 255, 255))
    draw_grid()
    draw_obstacles()
    draw_trails()   # ⭐ draw trails first
    draw_robots()
    pygame.display.flip()

pygame.quit()
