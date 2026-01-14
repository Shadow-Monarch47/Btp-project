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
        self.robot_id = robot_id
        self.color = color
        self.obstacles = obstacles
        self.global_path = self.a_star(start, goal, self.obstacles)
        self.start = start
        self.goal = goal
        self.was_blocked = False
        self.reached_goal = False
        self.in_narrow_path = False
        self.narrow_path_length = None
        self.priority = 0
        self.backtrack_point = None
        self.backtrack_path = []
        self.set_priority(robot_id)

    

#---------------------------------------------A-star starts here-----------------------------------------------------------

    def is_valid_position_for_obstacle(self, x, y, obstacles, grid_size, robot_size=1 ,):
        for dx in range(robot_size):
            for dy in range(robot_size):
                px, py = x + dx, y + dy
                if not (0 <= px < grid_size and 0 <= py < grid_size):
                    return False
                if (px, py) in obstacles:
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
                        self.is_valid_position_for_obstacle(orth2[0], orth2[1], obstacles, grid_size)):
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

        return []

#---------------------------------------------A-Star ends here----------------------------------------------------
#---------------------------------------------APF starts here-----------------------------------------------------
    def is_valid_position_for_robots_priority(
        self, x, y, grid_size,
        list_of_robots_in_avoidance_range,
        priority_dict):

        # Loop through robots in sensing area
        for name, (rx, ry) in list_of_robots_in_avoidance_range.items():
            if (rx, ry) == (x, y):
                # If robot has higher priority → block
                if priority_dict.get(name, 0) > self.priority:
                    return False
                # If lower priority → ALLOW cell
                else:
                    return True

        return True

    def get_neighbors_for_APF(self, pos, obstacles, grid_size, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {}):
        x, y = pos
        neighbors = []

        for dx in range(-1,2):
            for dy in range(-1,2):
                if dx == 0 and dy == 0:
                    continue

                new_x, new_y = x + dx, y + dy

                if not self.is_valid_position_for_obstacle(new_x, new_y, obstacles, grid_size):
                    continue

                if not self.is_valid_position_for_robots_priority(new_x, new_y, grid_size, list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range, priority_dict=priority_dict): #
                    continue 
        
                else:
                    neighbors.append((new_x, new_y))

        return neighbors
    

    def simple_apf_choose_next(self, pos, goal, obstacles, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {}):
        """
        Simple APF that applies attraction toward goal and repulsion from nearby robots.
        Rounds to nearest available 45° neighbor direction.
        """
        pos = np.array(pos, dtype=float)
        goal = np.array(goal, dtype=float)
        
        # Get valid neighbors
        neighbours = self.get_neighbors_for_APF(pos, obstacles, grid_size=55,
                                            list_of_robots_in_avoidance_range=list_of_robots_in_avoidance_range)
        
        
        
        if not neighbours:
            return tuple(pos.astype(int)) # No valid moves, Hold position
        
        # Attractive force toward goal
        F_att = (goal - pos)
        
        # Repulsive force from nearby robots
        F_rep = np.zeros(2, dtype=float)
        rep_radius = 4.0
        k_rep = 50.0

        higher_robots = {rname: rpos for rname, rpos in list_of_robots_in_avoidance_range.items() 
                        if priority_dict.get(rname, 0) > self.priority}
        
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
        best_idx = min(range(8), key=lambda i: abs(angle - DIR_ANGLES[i]))
        chosen_dir = DIRS[best_idx]
        
        dx, dy = chosen_dir
        final_pos = (int(pos[0] + dx), int(pos[1] + dy))
        
        # If chosen direction not available, pick nearest available neighbor
        if final_pos not in neighbours:
            def angle_to(npos):
                vec = np.array(npos) - pos
                return math.degrees(math.atan2(vec[1], vec[0]))
            final_pos = min(neighbours, key=lambda n: abs(angle - angle_to(n)))
        
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
            """Cell is blocked if out of bounds OR is an obstacle."""
            # Out of bounds check
            if not (0 <= x < grid_size and 0 <= y < grid_size):
                return True
            # Obstacle check
            if (x, y) in self.obstacles:
                return True
            return False
        
        
        
        # ========================================
        # CASE 1: HORIZONTAL MOVEMENT (dy == 0)
        # ========================================
        if dx != 0 and dy == 0:
            # Check cells above and below destination
            above = (next_x, next_y + 1)
            below = (next_x, next_y - 1)
            
            if is_blocked(above[0], above[1]) and is_blocked(below[0], below[1]):
                return True
        
        # ========================================
        # CASE 2: VERTICAL MOVEMENT (dx == 0)
        # ========================================
        elif dx == 0 and dy != 0:
            # Check cells left and right of destination
            left = (next_x - 1, next_y)
            right = (next_x + 1, next_y)
            
            if is_blocked(left[0], left[1]) and is_blocked(right[0], right[1]):
                return True
        
        # No narrow path detected
        return False
    
#---------------------------------------------Narrow path detector ends here---------------------------------------
#---------------------------------------------Narrow path APF stars here-------------------------------------------

    

    def narrow_path_apf_choose_next(self, pos, prev_pos , goal, obstacles, list_of_robots_in_avoidance_range : dict = {}, priority_dict : dict = {} ):
        
        neighbours = self.get_neighbors_for_narrow_path_APF(pos, obstacles, grid_size=55)

        if neighbours == []:
            return pos

        pos = np.array(pos, dtype=float)

        F_att = (goal - pos)

        # Repulsive force
        F_rep = np.zeros(2, dtype=float)
        rep_radius = 4.0
        k_rep = 5.0

        
        robots_to_avoid = {rname: rpos for rname, rpos in list_of_robots_in_avoidance_range.items() 
                        if priority_dict.get(rname, 0) > self.priority}
            
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


        # If no repulsion force (no higher priority robots), stay put
        if np.linalg.norm(F_rep) < 1e-6:
            return tuple(pos.astype(int))

        # Total force (pure repulsion for backtracking)
        F = F_rep + F_att

        # Angle of resultant force
        angle = math.degrees(math.atan2(F[1], F[0]))

        # Find nearest 45° direction
        best_idx = min(range(8), key=lambda i: abs(angle - DIR_ANGLES[i]))
        chosen_dir = DIRS[best_idx]

        # Convert selected direction to actual neighbor
        dx, dy = chosen_dir
        final_pos = (int(pos[0] + dx), int(pos[1] + dy))

        # If that neighbor is not available, fallback to nearest available one
        if final_pos not in neighbours:
            return pos

        return final_pos

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