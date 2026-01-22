from os import name
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from helping_functions import *
from algo2 import custom_algo
from obstacle_creator import ObstacleMapManager
import traceback
import random


#I'll analyze the code you've provided and help

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
interval_ms = 500 
max_limit_frames = 1000
initial , final = 24,10
frame_counter = 0
last_processed_frame = -1

#----------------------------------------------- Robot Configurations

robot_configs = [
    {"robot_id": "R_1", "start": (initial, initial), "goal": (final, final), "color": "red"},
    {"robot_id": "R_2", "start": (initial, final), "goal": (final, initial), "color": "green"},
    {"robot_id": "R_3", "start": (final, final), "goal": (initial, initial), "color": "blue"},                 
    {"robot_id": "R_4", "start": (final, initial), "goal": (initial, final), "color": "purple"},
]

robot_configs = [
    {"robot_id": "R_1", "start": (initial, initial), "goal": (initial, final), "color": "red"},
    # {"robot_id": "R_2", "start": (initial, final), "goal": (final, initial), "color": "green"},
    # {"robot_id": "R_3", "start": (final, final), "goal": (initial, initial), "color": "blue"},                 
    # {"robot_id": "R_4", "start": (final, initial), "goal": (final, final), "color": "purple"},
]

robot_configs = [
    {"robot_id": "R_1", "start": (27, 19), "goal": (20, 42), "color": "red"},
    {"robot_id": "R_2", "start": (27,42), "goal": (19,15), "color": "green"},
    {"robot_id": "R_3", "start": (27,17), "goal": (40,42), "color": "blue"},                 
    {"robot_id": "R_4", "start": (27,44), "goal": (39, 15), "color": "purple"},
]

# --------------------------------------------- OBSTACLE setup
obstacle_manager = ObstacleMapManager(
            grid_size=55,
            save_file="yielding_robot_obstacle_maps.json"
        )

obstacles = set(obstacle_manager.get_obstacles())

#----------------------------------------------- Robot generatior function

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
            "BackTrack_status": False,
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

#----------------------------------------------- Base plotter setup

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, GRID)
ax.set_ylim(0, GRID)
ax.set_xticks(range(GRID))
ax.set_yticks(range(GRID))
ax.grid(True)

# ---------------------------------------------- DRAW OBSTACLES 
for ox, oy in obstacles:
    ax.add_patch(Rectangle((ox, oy), 1, 1, color="grey"))

# ---------------------------------------------- Draw starting positions and trails of ROBOTS 

robot_rectangles = {}
robot_labels = {}
robot_trails = {}
trail_x = {}
trail_y = {}

for rid, details in Robot_details.items():
    x0, y0 = details["Global_path"][0]

    rect = Rectangle((x0, y0), 1, 1, color=robots[rid].color, alpha = 0.5)
    ax.add_patch(rect)
    robot_rectangles[rid] = rect

    label = ax.text(x0, y0, rid.replace("R_", "R"), color="black", fontsize=12, fontweight="bold")
    robot_labels[rid] = label

    trail_x[rid] = [x0 + 0.5]
    trail_y[rid] = [y0 + 0.5]
    trail, = ax.plot(trail_x[rid], trail_y[rid], color=robots[rid].color, linewidth=1.8)
    robot_trails[rid] = trail

#-------------------------------------------------- dictionaries to store current position and priorities of all robots

pos_all = {rid: Robot_details[rid]["Current_position"] for rid in Robot_details} 

priority_dict = {rid: Robot_details[rid]["Robot_priority"] for rid in Robot_details}

#---------------------------------------------------------- Robot Plotter function Defination

def move_robot(name,path, current_pos, rect, label, trail_x, trail_y):        
        
        x, y = path[-1]
        current_pos = (x, y)

        print(f"{name} moved to {current_pos}")
        

        rect.set_xy((x, y))
        label.set_position((x, y))

        if current_pos == Robot_details[name]["Goal"]:
            Robot_details[name]["Reached_Goal"] = True

        trail_x.append(x + 0.5)
        trail_y.append(y + 0.5)

        
        pos_all[name] = current_pos
        return current_pos

#------------------------------------------------------------------ MAIN CODE AND LOGIC STARTS FROM HERE, this function is repeated at each time interval

def update(frame):
    try:
    #----------------------------------------------Debug code, not important
        global last_processed_frame
        print(f"Frame: {frame}")
        print("*"*25)

        ax.set_title(f"Frame : {frame}",{'fontsize':25,'fontweight':10})

        if frame <= last_processed_frame:
            return (list(robot_rectangles.values()) + list(robot_labels.values()) + list(robot_trails.values()))
        last_processed_frame = frame

    #------------------------------------------------ THIS IS WHERE MAIN LOGIC OF THE CODE WILL RESIDE

        def algo_switch(name, pos, robots_to_avoid : dict, priority_dict : dict):

            goal = Robot_details[name]["Goal"]
            narrow_end = Robot_details[name]["Narrow_path_end"]
            A = robots_to_avoid
            B = Robot_details[name]["Was_blocked"]
            C = Robot_details[name]["Narrow_path_status"]
            
            if not A and not B and not C:        
                Robot_details[name]["Was_blocked"] = False                              
                planned_path = Robot_details[name]["Global_path"]
                next_index = Robot_details[name]["Global_path_frame_counter"] + 1
                
                if next_index < len(planned_path):
                    next_pos = planned_path[next_index]
                    print(f" {name}: Branch 1 ")
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["Actual_path_frame_counter"] += 1
                    Robot_details[name]["Global_path_frame_counter"] += 1
                    return next_pos
                else:
                    #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                    actual_paths[name].append(pos)
                    return pos
                
            

            if not A and not B and C:
                Robot_details[name]["Was_blocked"] = False
                planned_path = Robot_details[name]["Global_path"]
                next_index = Robot_details[name]["Global_path_frame_counter"] + 1
                
                if next_index < len(planned_path):
                    next_pos = planned_path[next_index]
                    print(f"  {name}: Branch 2 ")
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["Actual_path_frame_counter"] += 1
                    Robot_details[name]["Global_path_frame_counter"] += 1
                    return next_pos
                else:
                    #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                    actual_paths[name].append(pos)
                    return pos
                

                
            if not A and B and not C:
                Robot_details[name]["Was_blocked"] = False
                Robot_details[name]["Global_path"] = robots[name].a_star(pos, goal, obstacles)
                planned_path = Robot_details[name]["Global_path"]
                Robot_details[name]["Global_path_frame_counter"] = 0
                next_index = Robot_details[name]["Global_path_frame_counter"] + 1

                if next_index < len(planned_path):
                    next_pos = planned_path[1]
                    print(f"  {name}: Branch 3 ")
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["Actual_path_frame_counter"] += 1
                    Robot_details[name]["Global_path_frame_counter"] += 1
                    return next_pos
                else:
                    #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                    actual_paths[name].append(pos)
                    return pos
                

                
            if not A and B and C:
                Robot_details[name]["Was_blocked"] = False
                Robot_details[name]["Global_path"] = robots[name].a_star(pos, goal, obstacles)
                planned_path = Robot_details[name]["Global_path"]
                Robot_details[name]["Global_path_frame_counter"] = 0
                next_index = Robot_details[name]["Global_path_frame_counter"] + 1

                if next_index < len(planned_path):
                    next_pos = planned_path[1]
                    print(f"  {name}: Branch 4 ")
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["Actual_path_frame_counter"] += 1
                    Robot_details[name]["Global_path_frame_counter"] += 1
                    
                    return next_pos
                else:
                    #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                    actual_paths[name].append(pos)
                    return pos



            if A and not B and not C:
                Robot_details[name]["Was_blocked"] = True
                next_pos = robots[name].simple_apf_choose_next(pos,goal,obstacles,A,priority_dict)
                print(f"  {name}: Branch 5 ")
                actual_paths[name].append(next_pos)
                Robot_details[name]["Actual_path_frame_counter"] += 1
                return next_pos
            

            if (A and not B and C) or (A and B and C):
                Robot_details[name]["Was_blocked"] = True
                
                x_distance = Robot_details[name]["Narrow_path_end"][0] - Robot_details[name]["Narrow_path_start"][0]
                y_distance = Robot_details[name]["Narrow_path_end"][1] - Robot_details[name]["Narrow_path_start"][1]

                total_narrow_path_distance = abs(x_distance) + abs(y_distance) + 1
                

                current_distance = abs(Robot_details[name]["Narrow_path_start"][0] - pos[0]) + abs(Robot_details[name]["Narrow_path_start"][1] - pos[1]) + 1
                
                percent_narrow_path_covered = current_distance / total_narrow_path_distance * 100

                print(f"{name}: {int(percent_narrow_path_covered)}")

                if percent_narrow_path_covered >= 70:
                    Robot_details[name]["Narrow_path_priority"] = Robot_details[name]["Robot_priority"] + 1
                    narrow_priority_dict[name] = Robot_details[name]["Robot_priority"] + 1
                
                
                if 50<percent_narrow_path_covered<=70:
                    dx,dy = direction_dict[name]
                    straight_direction_list = [(1,0),(-1,0),(0,1),(0,-1)]
                    
                    straight_direction_list.remove((-dx,-dy))
                    

                    for other_name,other_pos in robots_to_avoid.items():
                        direction_of_other_robot = (other_pos[0] - pos[0], other_pos[1] - pos[1])
                        if direction_of_other_robot in straight_direction_list:
                            if Robot_details[name]["Remaining_global_path_length"] < Robot_details[other_name]["Remaining_global_path_length"] :
                                Robot_details[name]["Narrow_path_priority"] = Robot_details[name]["Robot_priority"] + 1 
                                narrow_priority_dict[name] = Robot_details[name]["Robot_priority"] + 1
                print(f"{name}: {Robot_details[name]["Remaining_global_path_length"]}")
                
            
                next_pos = robots[name].narrow_path_apf_choose_next(pos,narrow_end,obstacles,A,narrow_priority_dict)
                print(f"  {name}: Branch 8 ")
                actual_paths[name].append(next_pos)
                Robot_details[name]["Actual_path_frame_counter"] += 1
                return next_pos
            
            if A and B and not C:
                Robot_details[name]["Was_blocked"] = True
                next_pos = robots[name].simple_apf_choose_next(pos,goal,obstacles,A,priority_dict)
                print(f"  {name}: Branch 7 ")
                actual_paths[name].append(next_pos)
                Robot_details[name]["Actual_path_frame_counter"] += 1
                return next_pos
            

    #----------------------------------------------- Robots moved to the latest positions
        direction_dict = {}

        if frame > 0:
            for rid in Robot_details.keys():
                Robot_details[rid]["Current_position"] = move_robot(
                    rid,
                    actual_paths[rid],
                    Robot_details[rid]["Current_position"],
                    robot_rectangles[rid],
                    robot_labels[rid],
                    trail_x[rid],
                    trail_y[rid],
                )
                robot_trails[rid].set_data(trail_x[rid], trail_y[rid])

                x_direction = actual_paths[rid][-1][0] - actual_paths[rid][-2][0]
                y_direction = actual_paths[rid][-1][1] - actual_paths[rid][-2][1]
                direction_dict[rid] = (x_direction, y_direction)

        for rid in Robot_details.keys():
            curr_index = Robot_details[rid]["Global_path"].index(Robot_details[rid]["Current_position"]) if Robot_details[rid]["Current_position"] in Robot_details[rid]["Global_path"] else None
            remaining_path_length = len(Robot_details[rid]["Global_path"]) - curr_index - 1 if curr_index is not None else None
            Robot_details[rid]["Remaining_global_path_length"] = remaining_path_length

    #------------------------------------------------  avoidance range calculated for each one

        robots_in_avoidance_range = {}

        for rid in Robot_details.keys():
            # Build dict of "all other robots"
            other_positions = {
                other_rid: Robot_details[other_rid]["Current_position"]
                for other_rid in Robot_details.keys()
                if other_rid != rid
            }

            robots_in_avoidance_range[rid] = check_adjacent(
                rid,
                Robot_details[rid]["Current_position"],
                other_positions
            )

    #------------------------------------------------- new positions calculated and saved in a dict for conflict resolution

        next_pos_all = {}

        for rid in Robot_details.keys():
            next_pos_all[rid] = algo_switch(
                rid,
                Robot_details[rid]["Current_position"],
                robots_in_avoidance_range[rid],
                priority_dict
            )

    #-------------------------------------------------- Conflict resolution

        for name1 in list(Robot_details.keys()):
            for name2 in list(Robot_details.keys()):
                if name1 == name2:
                    continue
                    
                # Head-on collision: both want same cell
                if next_pos_all[name1] == next_pos_all[name2]:
                    if priority_dict[name1] < priority_dict[name2]:
                        # Lower priority waits
                        actual_paths[name1][-1] = pos_all[name1]
                        print(f"{name1} holded")

                    if priority_dict[name1] > priority_dict[name2]:
                        actual_paths[name2][-1] = pos_all[name2]
                        print(f"{name2} holded")

                if next_pos_all[name1] == pos_all[name2]:
                    if Robot_details[name1]["Narrow_path_status"] and Robot_details[name2]["Narrow_path_status"]:
                        if narrow_priority_dict[name1] < narrow_priority_dict[name2]:
                            x,y = direction_dict[name1][0], direction_dict[name1][1]
                            actual_paths[name1][-1] = (actual_paths[name1][-2][0] - x, actual_paths[name1][-2][1] - y)
                            print(f"{name1} reverted back")
                            continue
                            
                        if narrow_priority_dict[name1] < narrow_priority_dict[name2]:
                            actual_paths[name2][-1] = pos_all[name2]
                            print(f"{name2} holded")
                            continue

    #----------------------------------------------------- narrow path detector

        for rid, rob in robots.items():
            
            if len(actual_paths[rid]) >= 2:
                Robot_details[rid]["Narrow_path_status"]  = rob.narrow_path_detector(
                    actual_paths[rid][-2],
                    actual_paths[rid][-1]
                )
            
            if Robot_details[rid]["Narrow_path_status"] and Robot_details[rid]["Narrow_origin_marker"]:
                Robot_details[rid]["Narrow_origin_marker"] = False
                Robot_details[rid]["Narrow_path_start"] , Robot_details[rid]["Narrow_path_end"] = rob.update_narrow_path_bounds(actual_paths[rid][-2])
                

            if not Robot_details[rid]["Narrow_path_status"]:
                Robot_details[rid]["Narrow_origin_marker"] = True 
                Robot_details[rid]["Narrow_path_start"] , Robot_details[rid]["Narrow_path_end"] = None, None

            
        for rid,pos in pos_all.items():
            if actual_paths[rid][-1:-7:-1].count(pos) >= 3 and not Robot_details[rid]["Reached_goal"] :
                avail_positions = robots[rid].get_neighbors_for_APF(pos , obstacles , GRID , robots_in_avoidance_range[rid] , priority_dict)
                random_pos = random.choice(avail_positions)
                actual_paths[rid][-1] = random_pos

        for rid,pos in pos_all.items():
            if pos == Robot_details[rid]["Goal"]:
                Robot_details[rid]["Reached_goal"] = True


        for rid in Robot_details.keys():
           if Robot_details[rid]["Reached_goal"]:
                print(f"{rid} reached goal")



    #------------------------------------------------------ Return statment, not important

        print("*"*25)

        for rid,rob in robots.items():
            robots[rid].sync_from_plot(
                Robot_details[rid]["Global_path"],
                Robot_details[rid]["Was_blocked"],
                Robot_details[rid]["Current_position"],
                Robot_details[rid]["Narrow_path_status"],
                Robot_details[rid]["Narrow_path_start"],
                Robot_details[rid]["Narrow_path_end"],
            )

        return (list(robot_rectangles.values()) + list(robot_labels.values()) + list(robot_trails.values()))
    
    except Exception as e:
        print(e)
        traceback.print_exc()
        return (list(robot_rectangles.values()) + list(robot_labels.values()) + list(robot_trails.values()))

    


#-------------------------------------------------------- animation initialisation

ani = animation.FuncAnimation(
    fig,
    update,
    frames=max_limit_frames,
    interval=interval_ms,
    blit=False,
    repeat=False,
)

plt.show()