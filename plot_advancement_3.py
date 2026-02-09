from helping_functions import *
from algo2 import custom_algo
from obstacle_creator import ObstacleMapManager
import traceback
import random
import pygame
import numpy as np

#-----------------------------------------------Logger File code

import sys
from terminal_logger import TerminalLogger

LOG_FILE = "run_log.txt"

# 🔴 CLEAR previous run log
open(LOG_FILE, "w").close()

# 🔵 Redirect stdout/stderr
sys.stdout = TerminalLogger(LOG_FILE)
sys.stderr = sys.stdout

#----------------------------------------------- Base Variables

GRID = 55
initial , final = 24,10
CELL_SIZE = 18
WINDOW_SIZE = GRID * CELL_SIZE
FPS = 60
MOVE_DURATION = 0.6

robot_configs = [
    {"robot_id": "R_1", "start": (27, 19), "goal": (20, 42), "color": "red"},
    {"robot_id": "R_2", "start": (27,40), "goal": (19,15), "color": "green"},
    # {"robot_id": "R_3", "start": (27,17), "goal": (40,42), "color": "blue"},                 
    # {"robot_id": "R_4", "start": (27,44), "goal": (39, 15), "color": "purple"},
]

# --------------------------------------------- OBSTACLE setup
obstacle_manager = ObstacleMapManager(
            grid_size=55,
            save_file="yielding_robot_obstacle_maps.json"
        )

obstacles = set(obstacle_manager.get_obstacles())

def generate_robots(robot_configs):
    robots = {}
    Robot_details = {}
    priority_dict = {}
    actual_paths = {}
    narrow_priority_dict = {}

    for cfg in robot_configs:
        rid = cfg["robot_id"]
        start = cfg["start"]
        goal = cfg["goal"]
        color = cfg["color"]

        rob = custom_algo(robot_id=rid, start=start, goal=goal, color=color, obstacles = obstacles)
        robots[rid] = rob

        Robot_details[rid] = {
            "Start": rob.start,
            "Goal": rob.goal,
            "Global_path": rob.global_path,
            "Robot_priority": rob.priority,
            "Was_blocked": rob.was_blocked,
            "Narrow_path_status": rob.narrow_path_status,
            "Narrow_origin_marker": True, #
            "Narrow_path_start" : rob.narrow_entry,
            "Narrow_path_end" : rob.narrow_exit,
            "Narrow_path_priority": rob.priority,
            "Backtrack_status": False,
            "Reached_goal": rob.reached_goal,
            "A_star_calculated_for_narrow_path": False, #
            "Global_path_frame_counter": 0, #
            "Actual_path_frame_counter": 0 #
        }



        priority_dict[rid] = rob.priority
        actual_paths[rid] = [rob.global_path[0]]
        Robot_details[rid]["Current_position"] = tuple(rob.global_path[0])

        narrow_priority_dict[rid] = rob.priority

        Robot_details[rid]["Remaining_global_path_length"] = 0

    return robots, Robot_details, priority_dict, actual_paths, narrow_priority_dict

#----------------------------------------------- universal dictionaries with robot id as key to access details of robots all-together

robots, Robot_details, priority_dict, actual_paths , narrow_priority_dict = generate_robots(robot_configs)

# ================= Pygame INIT =================
pygame.init()
screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
pygame.display.set_caption("Simple Smooth Robot Animation with Trails")
clock = pygame.time.Clock()

# ================= DRAW GRID =================

def draw_grid():
    for i in range(GRID + 1):
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

# ================= DRAW GRID =================

def draw_obstacles():
    for (x, y) in obstacles:
        screen_x = x * CELL_SIZE
        screen_y = (GRID - 1 - y) * CELL_SIZE

        rect = pygame.Rect(
            screen_x,
            screen_y,
            CELL_SIZE,
            CELL_SIZE
        )
        pygame.draw.rect(screen, (60, 60, 60), rect)  # dark gray



