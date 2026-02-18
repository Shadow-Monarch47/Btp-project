import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from algo2 import custom_algo
from obs2 import ObstacleMapManager as obs1
from helping_functions import *
import traceback
import random
import itertools

#-----------------------------------------------Logger File code

import sys
from terminal_logger import TerminalLogger

LOG_FILE = "run_log_4.txt"

# 🔴 CLEAR previous run log
open(LOG_FILE, "w").close()

# 🔵 Redirect stdout/stderr
sys.stdout = TerminalLogger(LOG_FILE)
sys.stderr = sys.stdout

#----------------------------------------------- Base Variables

GRID = 55
interval_ms = 500 
initial , final = 0,54
frame_counter = 0
last_processed_frame = -1
frame_history = []
simulation_finished = False


#----------------------------------------------- Robot Configurations



# robot_configs = [
#     {"robot_id": "R_1", "start": (21, 13), "goal": (54, 54), "color": "red"},
#     {"robot_id": "R_2", "start": (33, 41), "goal": (0, 0), "color": "green"},
#     {"robot_id": "R_3", "start": (21, 41), "goal": (54, 0), "color": "blue"},
#     {"robot_id": "R_4", "start": (33, 13), "goal": (0, 54), "color": "purple"},
#     # {"robot_id": "R_5", "start": (27, 43), "goal": (54, 8), "color": "orange"},
    
# ]

robot_configs = [
    {"robot_id": "R_1", "start": (0, 0), "goal": (54, 54), "color": "red"},
    {"robot_id": "R_2", "start": (54,54), "goal": (0,0), "color": "green"},
    {"robot_id": "R_3", "start": (0, 54), "goal": (54, 0), "color": "blue"},
    {"robot_id": "R_4", "start": (54, 0), "goal": (0, 54), "color": "purple"},
    #{"robot_id": "R_5", "start": (8, 8), "goal": (54, 45), "color": "orange"},
    
]
# ---------------------------------------------- OBSTACLE setup
obstacle_manager = obs1(
            grid_size=55,
            #save_file="Maps/narrow_path.json"              
            save_file="Maps/maps3.json"
        )

obstacles = set(obstacle_manager.get_obstacles())

#----------------------------------------------- Robot generatior function

def generate_robots(robot_configs):
    robots = {}
    Robot_details = {}
    priority_dict = {}
    actual_paths = {}
    past_direction_dict = {}
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
            "Narrow_attractor": rob.narrow_attractor,
            "Astar_switch": False,
            "Apf_switch": False,
            "Backtrack_status": False,
            "Deadlock_permit": False,
            "Deadlock_permit_counter": 0,
            "Reached_goal": rob.reached_goal,
            "Past_direction": None, #
            "Global_path_frame_counter": 0, #
            "Actual_path_frame_counter": 0 #
        }



        priority_dict[rid] = rob.priority
        actual_paths[rid] = [rob.global_path[0]]
        Robot_details[rid]["Current_position"] = tuple(rob.global_path[0])

        past_direction_dict[rid] = Robot_details[rid]["Past_direction"]

        

        Robot_details[rid]["Remaining_global_path_length"] = 0

    return robots, Robot_details, priority_dict, actual_paths, past_direction_dict

#----------------------------------------------- universal dictionaries with robot id as key to access details of robots all-together

robots, Robot_details, priority_dict, actual_paths, past_direction_dict = generate_robots(robot_configs)

#----------------------------------------------- Base plotter setup

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(0, GRID)
ax.set_ylim(0, GRID)
ax.set_xticks(range(GRID))
ax.set_yticks(range(GRID))
ax.grid(True)

paused = False

def on_key(event):
    global paused
    if event.key == ' ':
        if paused:
            ani.event_source.start()
        else:
            ani.event_source.stop()
        paused = not paused

fig.canvas.mpl_connect('key_press_event', on_key)

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
    xg, yg = details["Goal"]

    rect = Rectangle((x0, y0), 1, 1, color=robots[rid].color, alpha = 0.5)
    goal_rect = Rectangle((xg, yg), 1, 1, color=robots[rid].color, alpha = 0.5)
    ax.add_patch(rect)
    ax.add_patch(goal_rect)
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

direction_dict = {}

#---------------------------------------------------------- Robot Plotter function Defination

def move_robot(name,path, rect, label, trail_x, trail_y):        
        
        x, y = path[-1]
        current_pos = (x, y)

        
        

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
        print("*"*100)
        print(" ")

        ax.set_title(f"Frame : {frame}",{'fontsize':25,'fontweight':10})

        if frame <= last_processed_frame:
            return (list(robot_rectangles.values()) + list(robot_labels.values()) + list(robot_trails.values()))
        last_processed_frame = frame

    #------------------------------------------------ THIS IS WHERE MAIN LOGIC OF THE CODE WILL RESIDE

        def algo_switch(name, pos, robots_to_avoid : dict, priority_dict : dict):
            goal = Robot_details[name]["Goal"]
            narrow_end = Robot_details[name]["Narrow_path_end"]
            Rob_priority = robots[name].priority
            A = robots_to_avoid
            B = Robot_details[name]["Was_blocked"]
            C = Robot_details[name]["Narrow_path_status"]

                
            if not A and not B and not C:                                           # Cross-Checked
                #Robot_details[name]["Was_blocked"] = False                              
                planned_path = Robot_details[name]["Global_path"]
                next_index = Robot_details[name]["Global_path_frame_counter"] + 1
                    
                if next_index < len(planned_path):
                    next_pos = planned_path[next_index]
                    print(f"{name}: Branch 1 ")
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["Actual_path_frame_counter"] += 1
                    Robot_details[name]["Global_path_frame_counter"] += 1
                    return next_pos
                else:
                    #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                    actual_paths[name].append(pos)
                    return pos
                
            if not A and not B and C:                                              #Cross-Checked
                    #Robot_details[name]["Was_blocked"] = False
                    planned_path = Robot_details[name]["Global_path"]
                    next_index = Robot_details[name]["Global_path_frame_counter"] + 1
                    
                    if next_index < len(planned_path):
                        next_pos = planned_path[next_index]
                        print(f"{name}: Branch 2 ")
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        Robot_details[name]["Global_path_frame_counter"] += 1
                        return next_pos
                    else:
                        #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                        actual_paths[name].append(pos)
                        return pos
                    

                    
            if not A and B and not C:                                              #Cross-Checked       
                    Robot_details[name]["Was_blocked"] = False
                    Robot_details[name]["Global_path"] = robots[name].a_star(pos, goal, obstacles)
                    planned_path = Robot_details[name]["Global_path"]
                    Robot_details[name]["Global_path_frame_counter"] = 0
                    next_index = Robot_details[name]["Global_path_frame_counter"] + 1

                    if next_index < len(planned_path):
                        next_pos = planned_path[1]
                        print(f"{name}: Branch 3 ")
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        Robot_details[name]["Global_path_frame_counter"] += 1
                        return next_pos
                    else:
                        #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                        actual_paths[name].append(pos)
                        return pos

            if not A and B and C:                                                   #Cross-Checked
                    Robot_details[name]["Was_blocked"] = False
                    Robot_details[name]["Global_path"] = robots[name].a_star(pos, goal, obstacles)
                    planned_path = Robot_details[name]["Global_path"]
                    Robot_details[name]["Global_path_frame_counter"] = 0
                    next_index = Robot_details[name]["Global_path_frame_counter"] + 1

                    if next_index < len(planned_path):
                        next_pos = planned_path[1]
                        print(f"{name}: Branch 4 ")
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        Robot_details[name]["Global_path_frame_counter"] += 1
                        
                        return next_pos
                    else:
                        #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                        actual_paths[name].append(pos)
                        return pos
                    
            if A and not B and not C:
                    print(f"{name}: Branch 5 ")                                               #Cross-Checked
                    
                    Robot_details[name]["Was_blocked"] = True
                    for other_name, other_pos in robots_to_avoid.items():
                        if Robot_details[other_name]["Narrow_path_status"]:
                            priority_dict[other_name] += 1  
                            Robot_details[name]["Apf_switch"] = True
                            break
                        
                        elif priority_dict[name] < priority_dict[other_name]:
                            Robot_details[name]["Apf_switch"] = True
                            break

                    if Robot_details[name]["Apf_switch"]:
                        next_pos = robots[name].simple_apf_choose_next(pos,goal,actual_paths[name][-2],obstacles,A,priority_dict)
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        Robot_details[name]["Apf_switch"] = False
                        print(f"{name}: Branch 5.1 ")
                        return next_pos
                    
                    else:
                        
                        Robot_details[name]["Global_path"] = robots[name].a_star(pos, goal, obstacles)
                        planned_path = Robot_details[name]["Global_path"]
                        Robot_details[name]["Global_path_frame_counter"] = 0
                        next_index = Robot_details[name]["Global_path_frame_counter"] + 1

                        if next_index < len(planned_path):
                            next_pos = planned_path[1]
                            print(f"{name}: Branch 5.2 ")
                            actual_paths[name].append(next_pos)
                            Robot_details[name]["Actual_path_frame_counter"] += 1
                            Robot_details[name]["Global_path_frame_counter"] += 1
                            return next_pos
                        else:
                            #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                            actual_paths[name].append(pos)
                            return pos
            
            if (A and not B and C) or (A and B and C):
                    Robot_details[name]["Was_blocked"] = True
                    
                    x_distance = Robot_details[name]["Narrow_path_end"][0] - Robot_details[name]["Narrow_path_start"][0]
                    y_distance = Robot_details[name]["Narrow_path_end"][1] - Robot_details[name]["Narrow_path_start"][1]

                    total_narrow_path_distance = abs(x_distance) + abs(y_distance) + 1
                    
                    print(f"{name}: Branch 6 ")

                    current_distance = abs(Robot_details[name]["Narrow_path_start"][0] - pos[0]) + abs(Robot_details[name]["Narrow_path_start"][1] - pos[1]) + 1
                    
                    percent_narrow_path_covered = current_distance / total_narrow_path_distance * 100
                    int(percent_narrow_path_covered)

                    print(f"{name}: {int(percent_narrow_path_covered)}% Narrow path covered")
                    

                    dx,dy = direction_dict[name]
                    forward_coordinates = (pos[0] + dx, pos[1] + dy)
                    reverse_coordinates = (pos[0] - dx, pos[1] - dy)
                    straight_direction_list = [(1,0),(-1,0),(0,1),(0,-1),(0,0)]

                    if len(robots_to_avoid) >= 2:

                        for (other_name1, other_pos1), (other_name2, other_pos2) in itertools.permutations(robots_to_avoid.items(), 2):
                            
                            if other_pos1 == forward_coordinates and other_pos2 == reverse_coordinates:
                                B1 , B2 , B0 = Robot_details[other_name1]["Backtrack_status"] , Robot_details[other_name2]["Backtrack_status"] , Robot_details[name]["Backtrack_status"]

                                if Robot_details[other_name1]["Narrow_path_status"] and Robot_details[other_name2]["Narrow_path_status"]:


                                    if not B0 and not B1 and not B2:
                                        print(f"{name}: Rare Branch 6.1.1 Triggered")
                                        break

                                    elif not B0 and not B1 and B2:
                                        print(f"{name}: Branch 6.1.2 Triggered")
                                        priority_dict[other_name1] -= 1
                                        priority_dict[other_name2] += 1
                                        break
                                    
                                    elif not B0 and B1 and not B2:
                                        print(f"{name}: Branch 6.1.3 Triggered")
                                        priority_dict[other_name1] += 1
                                        priority_dict[other_name2] -= 1
                                        break                

                                    elif not B0 and B1 and B2:
                                        print(f"{name}: Rare Branch 6.1.4 Triggered")
                                        break                 

                                    elif B0 and not B1 and not B2:
                                        print(f"{name}: Branch 6.1.5 Triggered")
                                        priority_dict[other_name1] += 1
                                        priority_dict[other_name2] -= 1
                                        break

                                    elif B0 and not B1 and B2:
                                        print(f"{name}: Branch 6.1.6 Triggered")
                                        priority_dict[other_name1] -= 1
                                        priority_dict[other_name2] += 1
                                        break

                                    elif B0 and B1 and not B2:
                                        print(f"{name}: Branch 6.1.7 Triggered")
                                        priority_dict[other_name1] += 1
                                        priority_dict[other_name2] -= 1
                                        break

                                    elif B0 and B1 and B2:
                                        print(f"{name}: Rare Branch 6.1.8 Triggered")
                                        break
                                        
                                elif Robot_details[other_name1]["Narrow_path_status"] and not Robot_details[other_name2]["Narrow_path_status"]:
                                    print(f"{name}: Branch 6.1.8 Triggered")
                                    priority_dict[other_name1] += 1
                                    priority_dict[other_name2] -= 1
                                    break

                                elif not Robot_details[other_name1]["Narrow_path_status"] and Robot_details[other_name2]["Narrow_path_status"]:
                                    print(f"{name}: Branch 6.1.9 Triggered")
                                    priority_dict[other_name1] -= 1
                                    priority_dict[other_name2] += 1
                                    break

                        next_pos, zero_force_check = robots[name].narrow_path_apf_choose_next(pos,narrow_end,obstacles,A,priority_dict)
                        if zero_force_check:
                            next_pos = forward_coordinates
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        return next_pos

                    else:

                        for other_name,other_pos in robots_to_avoid.items():
                            if Robot_details[name]["Narrow_path_status"] and not Robot_details[other_name]["Narrow_path_status"]:
                                priority_dict[other_name] -= 1
                                
                            if Robot_details[name]["Backtrack_status"] or Robot_details[other_name]["Backtrack_status"]:
                                
                                if not Robot_details[name]["Backtrack_status"] and other_pos == forward_coordinates :
                                    if Robot_details[other_name]["Backtrack_status"]:
                                        if direction_dict[name] == direction_dict[other_name]:
                                            print(f"{name}: Branch 6.2.1 ")
                                            priority_dict[other_name] -= 1
                                        else:
                                            print(f"{name}: Branch 6.2.2 ")
                                            priority_dict[other_name] += 1

                                if not Robot_details[name]["Backtrack_status"] and other_pos == reverse_coordinates :
                                        print(f"{name}: Branch 6.2.3 ")
                                        pass

                                if Robot_details[name]["Backtrack_status"] and other_pos == reverse_coordinates:
                                        
                                        if Robot_details[other_name]["Backtrack_status"]:
                                            print(f"{name}: Branch 6.2.4 ")
                                            pass # Some unique and rarest bugs might arrive here. But will leave for now.
                                        elif not Robot_details[other_name]["Backtrack_status"]:
                                            print(f"{name}: Branch 6.2.5 ")
                                            priority_dict[other_name] += 1
                                        pass
                                
                                if Robot_details[name]["Backtrack_status"] and other_pos == forward_coordinates:
                                    if Robot_details[other_name]["Backtrack_status"]:
                                        print(f"{name}: Branch 6.2.6 ")
                                        pass # Some unique and rarest bugs might arrive here. But will leave for now.
                                    elif not Robot_details[other_name]["Backtrack_status"]:
                                        print(f"{name}: Branch 6.2.7 ")
                                        priority_dict[other_name] -= 1

                            else:
                                continue
                        
                        next_pos, zero_force_check = robots[name].narrow_path_apf_choose_next(pos,narrow_end,obstacles,A,priority_dict)
                        if zero_force_check:
                            next_pos = forward_coordinates
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        return next_pos
                            
                        
                    if percent_narrow_path_covered < 30:
                        print(f"{name}: Branch 6.3.1 ")
                        
                        priority_dict[name] = Rob_priority - 1
                        
                    
                    
                    elif 30<=percent_narrow_path_covered<=70:
                        print(f"{name}: Branch 6.3.2 ")
                        for other_name,other_pos in robots_to_avoid.items():
                            direction_of_other_robot = (other_pos[0] - pos[0], other_pos[1] - pos[1])
                            try:
                                if direction_of_other_robot in straight_direction_list:
                                    if Robot_details[name]["Remaining_global_path_length"] < Robot_details[other_name]["Remaining_global_path_length"] :
                                        priority_dict[name] = Rob_priority + 1
                                    elif Robot_details[name]["Remaining_global_path_length"] == Robot_details[other_name]["Remaining_global_path_length"] :
                                        continue
                                    elif Robot_details[name]["Remaining_global_path_length"] > Robot_details[other_name]["Remaining_global_path_length"]:
                                        priority_dict[name] = Rob_priority - 1
                                        
                                print("Try block")
                            except:
                                 if robots[name].manhattan_dist(Robot_details[name]["Goal"] , pos) < robots[other_name].manhattan_dist(Robot_details[other_name]["Goal"] , other_pos):
                                    priority_dict[name] = Rob_priority + 1
                                 elif robots[name].manhattan_dist(Robot_details[name]["Goal"] , pos) == robots[other_name].manhattan_dist(Robot_details[other_name]["Goal"] , other_pos):
                                    continue
                                 else:
                                    priority_dict[name] = Rob_priority - 1
                                 print("Except block")

                    elif percent_narrow_path_covered >= 70:
                        print(f"{name}: Branch 6.3.3 ")
                        priority_dict[name] = Rob_priority + 1
                                    
                    print(f"{name}: {Robot_details[name]["Remaining_global_path_length"]} steps Remain")
                    print(f"{name}: Priority = {priority_dict[name]}")
                    next_pos, zero_force_check = robots[name].narrow_path_apf_choose_next(pos,narrow_end,obstacles,A,priority_dict)
                    if zero_force_check:
                        next_pos = forward_coordinates
                    actual_paths[name].append(next_pos)
                    Robot_details[name]["Actual_path_frame_counter"] += 1
                    return next_pos

            if A and B and not C:               
                    print(f"{name}: Branch 7 ")                                #Cross-Checked
                    Robot_details[name]["Was_blocked"] = True
                    for other_name, other_pos in robots_to_avoid.items():
                        if Robot_details[other_name]["Narrow_path_status"]:
                            priority_dict[other_name] += 1  
                            Robot_details[name]["Apf_switch"] = True
                            break
                        
                        elif priority_dict[name] < priority_dict[other_name]:
                            Robot_details[name]["Apf_switch"] = True
                            break

                    if Robot_details[name]["Apf_switch"]:
                        next_pos = robots[name].simple_apf_choose_next(pos,goal,actual_paths[name][-2],obstacles,A,priority_dict)
                        actual_paths[name].append(next_pos)
                        Robot_details[name]["Actual_path_frame_counter"] += 1
                        Robot_details[name]["Apf_switch"] = False
                        print(f"{name}: Branch 7.1 ")
                        return next_pos
                    
                    else:
                        Robot_details[name]["Was_blocked"] = False
                        Robot_details[name]["Global_path"] = robots[name].a_star(pos, goal, obstacles)
                        planned_path = Robot_details[name]["Global_path"]
                        Robot_details[name]["Global_path_frame_counter"] = 0
                        next_index = Robot_details[name]["Global_path_frame_counter"] + 1

                        if next_index < len(planned_path):
                            next_pos = planned_path[1]
                            print(f"{name}: Branch 7.2 ")
                            actual_paths[name].append(next_pos)
                            Robot_details[name]["Actual_path_frame_counter"] += 1
                            Robot_details[name]["Global_path_frame_counter"] += 1
                            return next_pos
                        else:
                            #print(f"  {name}: ELIF branch - end of path, staying at {pos}")
                            actual_paths[name].append(pos)
                            return pos
            

            
            

            
            

    #----------------------------------------------- Robots moved to the latest positions

        
        print("Moving Info")
        print("/"*50)
        if frame > 0:
            for rid in Robot_details.keys():
                
                Robot_details[rid]["Current_position"] = move_robot(
                    rid,
                    actual_paths[rid],
                    robot_rectangles[rid],
                    robot_labels[rid],
                    trail_x[rid],
                    trail_y[rid],
                )
                robot_trails[rid].set_data(trail_x[rid], trail_y[rid])

                print(f"{rid} : Moved to {Robot_details[rid]["Current_position"]}")
        print("/"*50)
        print()

        for name in Robot_details.keys():
            try:
                x_direction = actual_paths[name][-1][0] - actual_paths[name][-2][0]
                y_direction = actual_paths[name][-1][1] - actual_paths[name][-2][1]
            except:
                x_direction = 0
                y_direction = 0
            direction_dict[name] = (x_direction, y_direction)
            if direction_dict[name] == (0,0):
                direction_dict[name] = past_direction_dict[name]
                print(f"{name}: Holded inside Algo")
                print(f"Direction: {direction_dict[name]}")
            else:
                past_direction_dict[name] = direction_dict[name]
                print(f"Direction: {direction_dict[name]}")

    

    #----------------------------------------------------- Backtrack detector
        Backtrace_leader = {}
        print("Backtracing Status")
        print("/"*50)
        for rid in Robot_details.keys():
            if Robot_details[rid]["Narrow_path_status"]:
                if actual_paths[rid][-1] == actual_paths[rid][-4]:
                    Robot_details[rid]["Backtrack_status"] = True
                    
            else: 
                Robot_details[rid]["Backtrack_status"] = False
        
            print(f"{rid}: {Robot_details[rid]["Backtrack_status"]}")
            
        print("/"*50)
        print()

    #------------------------------------------------  avoidance range calculated for each one

        robots_in_avoidance_range = {}
        # print("Robots in range")
        # print("/"*50)
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

            #print(f"{rid}: has {robots_in_avoidance_range[rid]} in range")
        # print("/"*50)
        # print()

    #------------------------------------------------- new positions calculated and saved in a dict for conflict resolution

        next_pos_all = {}

        robot_ids = list(Robot_details.keys())

        avoidance_snapshot = {
            rid: robots_in_avoidance_range[rid].copy()
            for rid in robot_ids
        }

        priority_dict_copies = {rid : priority_dict.copy() for rid in robot_ids}

        print("Computed Position and Direction")
        print("/"*50)
        for rid in robot_ids:
            print("__"*12)
            next_pos_all[rid] = algo_switch(
                rid,
                Robot_details[rid]["Current_position"],
                avoidance_snapshot[rid],
                priority_dict_copies[rid]
            )
            
            print(f"{rid}: computed {next_pos_all[rid]} from algo switch")
            print("__"*12)
            print()
        print("/"*50)
        print()

        for rid in Robot_details.keys():
                curr_index = Robot_details[rid]["Global_path"].index(Robot_details[rid]["Current_position"]) if Robot_details[rid]["Current_position"] in Robot_details[rid]["Global_path"] else None
                remaining_path_length = len(Robot_details[rid]["Global_path"]) - curr_index - 1 if curr_index is not None else None
                Robot_details[rid]["Remaining_global_path_length"] = remaining_path_length


        

    #----------------------------------------------------- narrow path detector

        for rid, rob in robots.items():
            try: 
                Robot_details[rid]["Narrow_path_status"]  = rob.narrow_path_detector(
                        actual_paths[rid][-2],
                        actual_paths[rid][-1]
                    )
                if Robot_details[rid]["Narrow_path_status"] and Robot_details[rid]["Narrow_origin_marker"]:
                        Robot_details[rid]["Narrow_origin_marker"] = False
                        Robot_details[rid]["Narrow_path_start"] , Robot_details[rid]["Narrow_path_end"] = rob.update_narrow_path_bounds(actual_paths[rid][-2])
                        

                if not Robot_details[rid]["Narrow_path_status"]:
                    Robot_details[rid]["Narrow_origin_marker"] = True 
                    #Robot_details[rid]["Narrow_path_start"] , Robot_details[rid]["Narrow_path_end"] = None, None

                # if Robot_details[rid]["Narrow_path_status"]:
                #     print(f"{rid}: {Robot_details[rid]['Narrow_path_start']} , {Robot_details[rid]['Narrow_path_end']}")
                print(f"{rid}:- Narrow path status : {Robot_details[rid]["Narrow_path_status"]}")
            except:
                print(f"Some error occured in narrow path detector for {rid}")

        

    #-------------------------------------------------- Conflict resolution

        print("Conflict Resolution")
        print("/"*50)
        for pair in itertools.combinations(list(Robot_details.keys()), 2):
            name1, name2 = pair
            N1 , N2 = Robot_details[name1]["Narrow_path_status"] , Robot_details[name2]["Narrow_path_status"]
            B1 , B2 = Robot_details[name1]["Backtrack_status"] , Robot_details[name2]["Backtrack_status"]
            P1 , P2 = priority_dict[name1] , priority_dict[name2]
            r1 , r2 = False , False
            # r2 holds for r1 = True
            # r1 holds for r2 = True

            if next_pos_all[name1] == next_pos_all[name2]:
                print(f"Mid point conflict between {name1} and {name2}")
                # for other_name in robots_in_avoidance_range[name1].keys():    
                #     if next_pos_all[other_name] == pos_all[name1]:
                #         actual_paths[name2][-1] = pos_all[name2]
                #         next_pos_all[name2] = pos_all[name2]
                #         print(f"{name1}: Moved to {next_pos_all[name1]}")
                #         break


                if N1 and N2:
                    if not B1 and B2:
                        r2 = True
                        #Robot_details[name1]["Backtrack_status"] = True

                    elif B1 and not B2:
                        r1 = True
                        #Robot_details[name2]["Backtrack_status"] = True

                    elif not B1 and not B2:
                        if P1 < P2:
                            r2 = True
                            #Robot_details[name1]["Backtrack_status"] = True
                        elif P1 > P2:
                            r1 = True
                            #Robot_details[name2]["Backtrack_status"] = True

                    elif B1 and B2:
                        print(f"Backtrack conflict between {name1} and {name2}")


                elif not N1 and N2:
                    r2 = True
                         
                elif N1 and not N2:
                    r1 = True

                elif not N1 and not N2:
                    if P1 < P2:
                        r2 = True
                    elif P1 > P2:
                        r1 = True

                if r1:
                    neighbours = robots[name2].get_neighbors_for_Astar(pos_all[name2] , obstacles , GRID)
                    next_3_cells = get_next_three(pos_all[name2] , direction_dict[name2])
                    possible_points = list(set(neighbours).intersection(set(next_3_cells)))
                    valid_points = [point for point in possible_points if point not in pos_all.values()]
                    valid_points = [point for point in valid_points if point not in next_pos_all.values()]
                    
                    if not valid_points :
                        actual_paths[name2][-1] = pos_all[name2]
                        next_pos_all[name2] = pos_all[name2]

                    else:
                        next_pos_all[name2] = random.choice(valid_points)
                        actual_paths[name2][-1] = next_pos_all[name2]

                    print(f"{name2}: Moved to {next_pos_all[name2]}")

                elif r2:
                    neighbours = robots[name1].get_neighbors_for_Astar(pos_all[name1] , obstacles , GRID)
                    next_3_cells = get_next_three(pos_all[name1] , direction_dict[name1])
                    possible_points = list(set(neighbours).intersection(set(next_3_cells)))
                    valid_points = [point for point in possible_points if point not in pos_all.values()]
                    valid_points = [point for point in valid_points if point not in next_pos_all.values()]
                    
                    if not valid_points :
                        actual_paths[name1][-1] = pos_all[name1]
                        next_pos_all[name1] = pos_all[name1]

                    else:
                        next_pos_all[name1] = random.choice(valid_points)
                        actual_paths[name1][-1] = next_pos_all[name1]
                        
                    print(f"{name1}: Moved to {next_pos_all[name1]}")
                
        print("/"*50)
        print()
            
#________________________________________________________________________________________________________________________________________________________________________________________
        print("Deadlock Prevention")
        print("/"*50)
        random_pos_dict = {}
        for rid,pos in pos_all.items():
                if actual_paths[rid][-1:-8:-1].count(pos) >= 4 and not Robot_details[rid]["Reached_goal"] and Robot_details[rid]["Deadlock_permit"]:
                    print(f"{rid}: in deadlock, randomizing")
                    avail_positions = robots[rid].get_neighbors_for_deadlock(pos , obstacles , GRID , robots_in_avoidance_range[rid])
                    next_pos_list = list(next_pos_all.values())
                    next_pos_list.remove(next_pos_all[rid])
                    avail_positions = [pos for pos in avail_positions if pos not in next_pos_list]
                    avail_positions = [rand for rand in avail_positions if rand not in random_pos_dict.values()]
                    avail_positions = [rand for rand in avail_positions if rand not in actual_paths[rid][-1:-8:-1]]
                    try:
                        random_pos = random.choice(avail_positions)
                    except:
                        random_pos = pos
                        actual_paths[rid].pop()
                        Robot_details[rid]["Was_blocked"] = True
                        Robot_details[rid]["Deadlock_permit"] = False
                        continue
                    random_pos_dict[rid] = random_pos
                    actual_paths[rid][-1] = random_pos
                    Robot_details[rid]["Was_blocked"] = True
                    Robot_details[rid]["Deadlock_permit"] = False

                if not Robot_details[rid]["Deadlock_permit"]:
                    Robot_details[rid]["Deadlock_permit_counter"] +=1
                    if Robot_details[rid]["Deadlock_permit_counter"] == 5:
                        Robot_details[rid]["Deadlock_permit"] = True
                        Robot_details[rid]["Deadlock_permit_counter"] = 0

        print("/"*50)
        print()



        


    
        
        
        
        
        for rid,pos in pos_all.items():
            if pos == Robot_details[rid]["Goal"]:
                Robot_details[rid]["Reached_goal"] = True


    #------------------------------------------------------ Return statment, not important

        print("*"*100)

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
    frames=itertools.count(),
    interval=interval_ms,
    blit=False,
    repeat=False,
)

plt.show()

