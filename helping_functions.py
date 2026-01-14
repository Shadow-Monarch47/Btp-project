

def check_adjacent(name, posA, other_positions):
    list_of_robots_in_avoidance_range = {} # store detected robots in vision range
    x, y = posA
    adj_offsets = [
        (-1, -1), (-1, 0), (-1, 1),
        (0, -1)      ,     (0, 1),
        (1, -1),  (1, 0),  (1, 1)
    ]

    for dx, dy in adj_offsets: 
        for other_name, (ox, oy) in other_positions.items():
            if (x + dx, y + dy) == (ox, oy):
                list_of_robots_in_avoidance_range[other_name] = (ox, oy)
                break
    
        # print(f" {name} sees {list_of_robots_in_avoidance_range}")
    return list_of_robots_in_avoidance_range 



def check_visible(name, posA, other_positions):
    list_of_robots_in_visible_range = {} # store detected robots in vision range
    x, y = posA
    visible_offsets = [
        (i,j) for i in [-2, -1, 0, 1, 2] for j in [-2, -1, 0, 1, 2] if not (i == 0 and j == 0)
    ]
    for dx, dy in visible_offsets: 
        for other_name, (ox, oy) in other_positions.items():
            if (x + dx, y + dy) == (ox, oy):
                list_of_robots_in_visible_range[other_name] = (ox, oy)
                break
    return list_of_robots_in_visible_range



