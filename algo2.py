import math
import numpy as np
from heapq import heappush, heappop


DIRS = [
    (1, 0),   # 0°
    (1, 1),   # 45°
    (0, 1),   # 90°
    (-1, 1),  # 135°
    (-1, 0),  # 180°
    (-1, -1), # -135°
    (0, -1),  # -90°
    (1, -1)   # -45°
]

DIR_ANGLES = [0, 45, 90, 135, 180, -135, -90, -45]



class custom_algo:
    def __init__(self, robot_id , start , goal, color, obstacles: set):
        self.obstacles = obstacles

        # variables linked to dictionary

        self.start = start
        self.goal = goal
        self.global_path = self.a_star(start, goal, self.obstacles)
        self.priority = 0
        self.was_blocked = False
        self.narrow_path_status = False
        self.narrow_entry = None
        self.narrow_exit = None
        entry = self.get_first_narrow_entry()
        self.narrow_attractor = entry if entry is not None else self.goal
        self.current_pos = start
        self.reached_goal = False

        # independent variables
        
        self.robot_id = robot_id
        self.color = color
        
        self.backtrack_point = None
        self.backtrack_path = []
        
        self.set_priority(robot_id)

    def sync_from_plot(self, global_path, block, current_pos, narrow_active, narrow_start, narrow_end ):
        """
        HARD RULES:
        - No logic
        - No conditions
        - No recomputation
        - Just assignment
        """

        self.current_pos = tuple(current_pos)
        self.was_blocked = bool(block)
        self.narrow_path_status = bool(narrow_active)
        self.narrow_entry = narrow_start
        self.narrow_exit = narrow_end
        self.global_path = global_path
    

#---------------------------------------------A-star starts here-----------------------------------------------------------

    def is_valid_position_for_obstacle(self, x, y, obstacles, grid_size):
                
        if not (0 <= x < grid_size and 0 <= y < grid_size):
            return False
        if (x, y) in obstacles:
            return False
        
        return True
    

    def get_neighbors_for_Astar(self, pos, obstacles, grid_size):
        x, y = pos
        neighbors = []

        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue

                if dx != 0 and dy != 0:
                    orth1 = (x + dx, y)
                    orth2 = (x, y + dy)

                    if (self.is_valid_position_for_obstacle(orth1[0], orth1[1], obstacles, grid_size) and
                        self.is_valid_position_for_obstacle(orth2[0], orth2[1], obstacles, grid_size)):    #diagonal cell valid 
                        neighbors.append((new_x, new_y))
                        
                else:
                    neighbors.append((new_x, new_y))

        return neighbors


    def a_star(self, start, goal, obstacles, grid_size=55):
        start = tuple(start)
        goal = tuple(goal)

        open_set = []
        heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_dist(start, goal)}

        while open_set:
            current = heappop(open_set)[1]

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for neighbor in self.get_neighbors_for_Astar(current, obstacles, grid_size):
                tentative = g_score[current] + self.dist(current, neighbor)

                if neighbor not in g_score or tentative < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f_score[neighbor] = tentative + self.manhattan_dist(neighbor, goal)
                    heappush(open_set, (f_score[neighbor], neighbor))

            #print global path



        return []

#---------------------------------------------A-Star ends here----------------------------------------------------
#---------------------------------------------APF starts here-----------------------------------------------------
    def is_valid_position_for_robots_priority(
        self, x, y, grid_size,
        list_of_robots_in_avoidance_range,
        priority_dict = {}):

        # Loop through robots in sensing area
        if priority_dict:
            for name, (rx, ry) in list_of_robots_in_avoidance_range.items():
                if (rx, ry) == (x, y):
                    # If robot has higher priority → block
                    if priority_dict.get(name, 0) > priority_dict[self.robot_id]:
                        return False
                    # If lower priority → ALLOW cell
                    else:
                        return True
                    
        else:
            for (rx, ry) in list_of_robots_in_avoidance_range.values():
                if (rx, ry) == (x, y):
                    return False
                else:
                    return True

        return True

    def get_neighbors_for_APF(self, pos, obstacles, grid_size, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {}):
        x, y = pos
        neighbors = []

        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue


                elif not self.is_valid_position_for_robots_priority(new_x, new_y, grid_size, list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range, priority_dict=priority_dict): #
                    continue 

                elif dx != 0 and dy != 0:
                    orth1 = (x + dx, y)
                    orth2 = (x, y + dy)

                    if (self.is_valid_position_for_obstacle(orth1[0], orth1[1], obstacles, grid_size) and
                        self.is_valid_position_for_obstacle(orth2[0], orth2[1], obstacles, grid_size)):    #diagonal cell valid 
                        neighbors.append((new_x, new_y))
        
                else:
                    neighbors.append((new_x, new_y))

        return neighbors
    

    def simple_apf_choose_next(self, pos, goal, prev_pos, obstacles, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {}):
        """
    Simple APF that applies attraction toward goal and repulsion from nearby robots.
    Rounds to nearest available 45° neighbor direction.
        """
        pos = np.array(pos, dtype=float)
        goal = np.array(goal, dtype=float)
    
    # Get valid neighbors
        neighbours = self.get_neighbors_for_APF(pos, obstacles, grid_size=55,
                                        list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range,
                                        priority_dict=priority_dict)
    
        neighbours.remove(prev_pos) if prev_pos in neighbours else neighbours
        print(f"Neighbours: {neighbours}")
        
    
    # Attractive force toward goal
        F_att = (goal - pos)
    
    # Repulsive force from nearby robots
        F_rep = np.zeros(2, dtype=float)
        rep_radius = 4.0
        k_rep = 50.0

        higher_robots = {rname: rpos for rname, rpos in list_of_robots_in_avoidance_range.items() 
                    if priority_dict.get(rname, 0) > priority_dict[self.robot_id]}
        
        if not neighbours:
            if prev_pos not in higher_robots.values():
                return prev_pos
            
    
        for robot_name, (rx, ry) in higher_robots.items():
            rob = np.array([rx, ry], dtype=float)
            dvec = pos - rob
            dist = np.linalg.norm(dvec)
            if dist >= 1e-6 and dist <= rep_radius:
                F_rep += k_rep * (1.0 / (dist**2)) * (dvec / dist)
    
    # Total force
        F = F_att + F_rep
        angle = math.degrees(math.atan2(F[1], F[0]))
    
    # Find nearest 45° direction
        def angular_distance(a, b):
            diff = abs(a - b) % 360
            return min(diff, 360 - diff)
        
        best_idx = min(range(8),key=lambda i: angular_distance(angle, DIR_ANGLES[i]))
        chosen_dir = DIRS[best_idx]
    
        dx, dy = chosen_dir
        final_pos = (int(pos[0] + dx), int(pos[1] + dy))
    
    # If chosen direction not available, pick nearest available neighbor
        if final_pos not in neighbours:
            def angle_to(npos):
                vec = np.array(npos) - pos
                return math.degrees(math.atan2(vec[1], vec[0]))
            final_pos = min(neighbours, key=lambda n: angular_distance(angle, angle_to(n)))
    
    # FIXED: Always return regular Python tuple
        return (int(final_pos[0]), int(final_pos[1]))

    
#---------------------------------------------APF ends here-------------------------------------------------------------------------------------
#---------------------------------------------Narrow path detector starts here------------------------------------------------------------------

    def narrow_path_detector(self, current_pos, next_pos, grid_size=55):
        """
        Detects if the robot is entering a narrow one-way passage.
        
        Args:
            name: Robot name (for debugging/logging)
            current_pos: (x, y) current position
            next_pos: (x, y) next intended position
            grid_size: Grid dimensions (default 55)
        
        Returns:
            bool: True if entering narrow passage, False otherwise
        """
        curr_x, curr_y = current_pos
        next_x, next_y = next_pos
        
        # Calculate movement direction
        dx = next_x - curr_x
        dy = next_y - curr_y
        
        # Helper function to check if a cell is blocked
        def is_blocked(x, y):
            if not (0 <= x < grid_size and 0 <= y < grid_size):
                return True
            return (x, y) in self.obstacles

        def diagonal_block_degree(x, y):
            directions = [(1,1),(1,-1),(-1,1),(-1,-1)]
            count = 0
            for dx, dy in directions:
                if is_blocked(x + dx, y + dy):
                    count += 1
            return count
        
        def free_degree(x, y):
            directions = [(1,0),(0,-1),(-1,0),(0,1)]
            count = 0
            for dx, dy in directions:
                if not is_blocked(x + dx, y + dy):
                    count += 1
            return count
        
        
        if not self.narrow_path_status:      # First point of narrow path must have both sides blocked
        
            # ========================================
            # CASE 1: HORIZONTAL MOVEMENT (dy == 0)
            # ========================================
            if dx != 0 and dy == 0:
                # Check cells above and below destination
                above = (curr_x, curr_y + 1)
                below = (curr_x, curr_y - 1)
                
                if is_blocked(above[0], above[1]) and is_blocked(below[0], below[1]):
                    return True
            

            # ========================================
            # CASE 2: VERTICAL MOVEMENT (dx == 0)
            # ========================================
            elif dx == 0 and dy != 0:
                # Check cells left and right of destination
                left = (curr_x - 1, curr_y)
                right = (curr_x + 1, curr_y)
                
                if is_blocked(left[0], left[1]) and is_blocked(right[0], right[1]):
                    return True
                
            elif dx == 0 and dy == 0:
                return False

        else:
            
            above = (next_x, next_y + 1)
            below = (next_x, next_y - 1)
            left = (next_x - 1, next_y)
            right = (next_x + 1, next_y)
            
            
            # ========================================
            # CASE 1: HORIZONTAL MOVEMENT (dy == 0)
            # ========================================
            if dx != 0 and dy == 0:
                
                if not is_blocked(above[0], above[1]) and not is_blocked(below[0], below[1]):
                    if not is_blocked(left[0], left[1]) and not is_blocked(right[0], right[1]):
                        if diagonal_block_degree(next_x, next_y) < 4:
                            return False
                
                return True
            
            # ========================================
            # CASE 2: VERTICAL MOVEMENT (dx == 0)
            # ========================================
            if dx == 0 and dy != 0:
                
                if not is_blocked(left[0], left[1]) and not is_blocked(right[0], right[1]):
                    if not is_blocked(above[0], above[1]) and not is_blocked(below[0], below[1]):
                        if diagonal_block_degree(next_x, next_y) < 4:
                            return False
                        
                return True
                
            if dx == 0 and dy == 0:
                print("Holded inside narrow path")
                return True

        
        # No narrow path detected
        return False
    
    # def update_narrow_path_bounds(self, current_pos, grid_size=55):
    #     """
    #     Finds start and end of the narrow path segment on the GLOBAL PATH.
    #     Sets entry/exit to None when not in narrow path.
    #     """

    #     path = self.global_path
    #     if not path or current_pos not in path:
    #         self.narrow_entry = None
    #         self.narrow_exit = None
    #         self.narrow_path_status = False
    #         return None, None

    #     curr_idx = path.index(current_pos)

    #     def is_narrow_cell(p, nxt):
    #         x, y = p
    #         nx, ny = nxt
    #         dx, dy = nx - x, ny - y

    #         def blocked(cx, cy):
    #             if not (0 <= cx < grid_size and 0 <= cy < grid_size):
    #                 return True
    #             return (cx, cy) in self.obstacles

    #         # vertical corridor
    #         if dx == 0 and dy != 0:
    #             return blocked(nx - 1, ny) or blocked(nx + 1, ny)

    #         # horizontal corridor
    #         if dx != 0 and dy == 0:
    #             return blocked(nx, ny - 1) or blocked(nx, ny + 1)

    #         return False

    #     # ---------- FIND ENTRY ----------
    #     entry_idx = 0
    #     for i in range(curr_idx, len(path) - 1):
    #         if is_narrow_cell(path[i], path[i + 1]):
    #             entry_idx = i
    #             break

    #     if entry_idx is None:
    #         self.narrow_entry = None
    #         self.narrow_exit = None
    #         self.narrow_path_status = False
    #         return None, None

    #     # ---------- FIND EXIT ----------
    #     exit_idx = entry_idx
    #     for i in range(entry_idx, len(path) - 1):
    #         if not is_narrow_cell(path[i], path[i + 1]):
    #             break
    #         exit_idx = i + 1

    #     self.narrow_entry = path[entry_idx]
    #     self.narrow_exit = path[exit_idx]
    #     self.narrow_path_status = True

    #     return self.narrow_entry, self.narrow_exit

    def update_narrow_path_bounds(self, current_pos, grid_size=55):
        """
        Robust narrow path detection using node-degree logic.

        - Uses global_path
        - Starts from current_pos
        - Scans forward to find corridor exit
        - If current_pos not in global_path, recomputes temporary path
        """

        # ---------------------------------------------------
        # SAFETY: If current_pos not in global_path
        # ---------------------------------------------------
        if not self.global_path or current_pos not in self.global_path:
            temp_path = self.a_star(current_pos, self.goal, self.obstacles, grid_size)
            if not temp_path:
                self.narrow_entry = None
                self.narrow_exit = None
                self.narrow_path_status = False
                return None, None
            path = temp_path
        else:
            path = self.global_path

        # ---------------------------------------------------
        # Degree logic helper
        # ---------------------------------------------------
        def is_blocked(x, y):
            if not (0 <= x < grid_size and 0 <= y < grid_size):
                return True
            return (x, y) in self.obstacles

        def diagonal_block_degree(x, y):
            directions = [(1,1),(1,-1),(-1,1),(-1,-1)]
            count = 0
            for dx, dy in directions:
                if is_blocked(x + dx, y + dy):
                    count += 1
            return count
        
        def free_degree(x, y):
            directions = [(1,0),(0,-1),(-1,0),(0,1)]
            count = 0
            for dx, dy in directions:
                if not is_blocked(x + dx, y + dy):
                    count += 1
            return count

        # ---------------------------------------------------
        # Find index of current position
        # ---------------------------------------------------
        try:
            curr_idx = path.index(current_pos)
        except ValueError:
            self.narrow_entry = None
            self.narrow_exit = None
            self.narrow_path_status = False
            return None, None

        # ---------------------------------------------------
        # Mark entry
        # ---------------------------------------------------
        entry_idx = curr_idx

        # ---------------------------------------------------
        # Scan forward until corridor ends
        # ---------------------------------------------------
        exit_idx = curr_idx

        for i in range(curr_idx + 1, len(path)):
            x, y = path[i]
            
            if not is_blocked(x-1,y) and not is_blocked(x+1,y):
                if not is_blocked(x,y-1) and not is_blocked(x,y+1):
                    if diagonal_block_degree(x,y) < 4:
                        break
                
            exit_idx = i

            
            
            

        # ---------------------------------------------------
        # Set class variables
        # ---------------------------------------------------
        self.narrow_entry = path[entry_idx]
        self.narrow_exit = path[exit_idx]
        self.narrow_path_status = True

        return self.narrow_entry, self.narrow_exit



    
#---------------------------------------------Narrow path detector ends here---------------------------------------
#---------------------------------------------Narrow path APF stars here-------------------------------------------

    def get_neighbors_for_narrow_path_APF(self, pos, obstacles, grid_size, list_of_robots_in_avoidance_range, priority_dict):
        x, y = pos
        neighbors = []

        for dx in range(-1,2):
            for dy in range(-1,2):
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue 

                elif not self.is_valid_position_for_robots_priority(new_x, new_y, grid_size, list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range, priority_dict=priority_dict): #
                    continue 

                elif dx != 0 and dy != 0:
                    orth1 = (x + dx, y)
                    orth2 = (x, y + dy)

                    if (self.is_valid_position_for_obstacle(orth1[0], orth1[1], obstacles, grid_size) and
                        self.is_valid_position_for_obstacle(orth2[0], orth2[1], obstacles, grid_size)):    #diagonal cell valid 
                        neighbors.append((new_x, new_y))
        
                else:
                    neighbors.append((new_x, new_y))
        return neighbors

    def narrow_path_apf_choose_next(self, pos, goal, obstacles, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {}):
        grid_size = 55
        neighbours = self.get_neighbors_for_narrow_path_APF(pos, obstacles, grid_size,list_of_robots_in_avoidance_range, priority_dict)

        if not neighbours:
        # FIXED: Convert pos to regular tuple
            if isinstance(pos, (list, np.ndarray)):
                return (int(pos[0]), int(pos[1])), False
            return pos, False

        pos = np.array(pos, dtype=float)
        goal = np.array(goal, dtype=float)  # FIXED: Ensure goal is numpy array

        F_att = (goal - pos)

    # Repulsive force
        F_rep = np.zeros(2, dtype=float)
        rep_radius = 4.0
        k_rep = 5.0

        robots_to_avoid = {rname: rpos for rname, rpos in list_of_robots_in_avoidance_range.items() 
                    if priority_dict.get(rname, 0) > priority_dict[self.robot_id]}
        
    # Find robot with highest priority from robots_to_avoid
        highest_priority_robot = max(robots_to_avoid.items(), 
                                key=lambda item: priority_dict.get(item[0], 0))[0] if robots_to_avoid else None
        
        for robot_name, (rx, ry) in robots_to_avoid.items():
            if robot_name == highest_priority_robot:
                rob = np.array([rx, ry], dtype=float)
                dvec = pos - rob
                dist = np.linalg.norm(dvec)
                if dist >= 1e-6 and dist <= rep_radius:
                    F_rep += k_rep * (1.0 / (dist**2)) * (dvec / dist)

        


    # Total force (pure repulsion for backtracking)
        F = F_rep + F_att
        if F[0] == 0.0 and F[1] == 0.0:
            return pos, True

    # Angle of resultant force
        angle = math.degrees(math.atan2(F[1], F[0]))

    # Find nearest 45° direction
        def angular_distance(a, b):
            diff = abs(a - b) % 360
            return min(diff, 360 - diff)
        
        best_idx = min(range(8),key=lambda i: angular_distance(angle, DIR_ANGLES[i]))
        chosen_dir = DIRS[best_idx]

    # Convert selected direction to actual neighbor
        dx, dy = chosen_dir
        final_pos = (int(pos[0] + dx), int(pos[1] + dy))

    # If that neighbor is not available, select the next nearest neighbor

        if final_pos not in neighbours:
            def angle_to(npos):
                vec = np.array(npos) - pos
                return math.degrees(math.atan2(vec[1], vec[0]))
            final_pos = min(neighbours, key=lambda n: angular_distance(angle,angle_to(n)))

        

        return (int(final_pos[0]), int(final_pos[1])) , False

#---------------------------------------------Narrow path APF ends here-------------------------------------------
#---------------------------------------------Deadlock neighbours starts here--------------------------------------

    def get_neighbors_for_deadlock(self, pos, obstacles, grid_size, list_of_robots_in_avoidance_range : dict = {}):
        x, y = pos
        neighbors = []

        for dx in range(-1,2):
            for dy in range(-1,2):
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue

                if not self.is_valid_position_for_robots_priority(new_x, new_y, grid_size, list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range): 
                    continue 

                # if dx != 0 and dy != 0:
                #     orth1 = (x + dx, y)
                #     orth2 = (x, y + dy)

                #     if (self.is_valid_position_for_obstacle(orth1[0], orth1[1], obstacles, grid_size) and
                #         self.is_valid_position_for_obstacle(orth2[0], orth2[1], obstacles, grid_size)):    #diagonal cell valid 
                #         neighbors.append((new_x, new_y))
        
                else:
                    neighbors.append((new_x, new_y))

        print(True)

        return neighbors

#---------------------------------------------Deadlock neighbours ends here---------------------------------------

    def get_first_narrow_entry(self, grid_size=55):
        """
        Returns the starting coordinate of the FIRST narrow path
        segment in the global path.

        Does NOT modify any class variables.
        Returns None if no narrow segment exists.
        """

        path = self.global_path
        if not path or len(path) < 2:
            return None

        def is_narrow_cell(p, nxt):
            x, y = p
            nx, ny = nxt
            dx, dy = nx - x, ny - y

            def blocked(cx, cy):
                if not (0 <= cx < grid_size and 0 <= cy < grid_size):
                    return True
                return (cx, cy) in self.obstacles

            # vertical corridor
            if dx == 0 and dy != 0:
                return blocked(nx - 1, ny) or blocked(nx + 1, ny)

            # horizontal corridor
            if dx != 0 and dy == 0:
                return blocked(nx, ny - 1) or blocked(nx, ny + 1)

            return False

        for i in range(len(path) - 1):
            if is_narrow_cell(path[i], path[i + 1]):
                return path[i]

        return None


#--------------------------------------------Extra functions start here--------------------------------------------

    def priority_resolution(self, name, p1 , other_name, p2):

        if p1 > p2:
            return True
        else:
             return False
        
    def set_priority(self,robot_id):
        robo_no = int(robot_id.split('_')[1])
        self.priority = float(1/robo_no)

    def dist(self,a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def manhattan_dist(self,a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    

    