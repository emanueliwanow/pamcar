import itertools
import math

# Distance between two given doors
def distance(door1, door2):
    return math.sqrt((math.pow(door1[0] - door2[0], 2) + math.pow(door1[1] - door2[1], 2)))

# Overall distance of a given path
def global_distance(path, doors):
    global_distance = 0
    for i in range(len(path) - 1):
        global_distance += distance(doors[path[i]], doors[path[i + 1]])
    global_distance += distance(doors[path[-1]], doors[path[0]])  # Coming back to initial point
    return global_distance

# Main algorithm for the TSP
def traveling_salesman_problem(doors):
    num_doors = len(doors)
    all_permutations = itertools.permutations(range(num_doors)) # Generate all possible distances

    min_distance = float('inf')
    best_path = None

    for permutation in all_permutations:
        current_distance = global_distance(permutation, doors)
        print("Current path:", permutation, " and distance for it:", current_distance)
        if current_distance < min_distance:
            min_distance = current_distance
            best_path = permutation

    return best_path, min_distance

# Example for our problem :
#doors_coordinates = [(0, 0), (3, 2), (2, 4), (6, 1)]
#best_path, min_distance = traveling_salesman_problem(doors_coordinates)

#print("Optimal path:", best_path)
#print("Distance using this path:", min_distance)