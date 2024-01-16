from TSP import *

# Example for our problem :
doors_coordinates = [(0, 0), (3, 2), (2, 4), (6, 1)]
best_path, min_distance = traveling_salesman_problem(doors_coordinates)

print("Optimal path:", best_path)
print("Distance using this path:", min_distance)