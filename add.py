robots_to_avoid = {
    "R1": (10, 5),
    # "R2": (3, 7),
    # "R3": (8, 2)
}

from itertools import combinations, permutations

for (other_name1, other_pos1), (other_name2, other_pos2) in permutations(robots_to_avoid.items(), 2):
    print(True)
    print(other_name1)
    print(5)