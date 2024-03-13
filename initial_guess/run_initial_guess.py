from casadi import *
from math import pi
import numpy as np
import os
os.path.join('../motion_planning', 'doors.py')
os.path.join('../motion_planning', 'opc_problem.py')
from doors import doors1, doors2, doors3, middle_points, middle_points_theta
from opc_problem import *
from OPC_resolution_door_to_door import *
from TSP import *
from save_data import *
# ---- post-processing ------
from pylab import plot, step, figure, legend, show, spy, title, xlabel, ylabel, annotate
import matplotlib.pyplot as plt

""" ---- Defining Environment ---- """
doors = doors1


""" ---- Defining doors related to the initial guess problem ---- """
# Creating arrays with zeros
middle_points = np.zeros((2,len(doors)))
middle_points_theta = np.zeros((1,len(doors)))
points = np.zeros((2,2*len(doors)))

i = 0
j = 0
for key, value in doors.items():
    # Getting x,y coordinates from doors
    x1 = value[0][0]
    y1 = value[0][1]
    x2 = value[1][0]
    y2 = value[1][1]

    # Getting middle point inside door
    middle_x = (x1+x2)/2
    middle_y = (y1+y2)/2

    # Putting coordinates in array for visualization
    points[:,j] = [x1,y1]
    points[:,j+1] = [x2,y2]
    middle_points[:,i] = [middle_x,middle_y]

    # Getting entrance angle in each door
    middle_points_theta[0,i] = (pi/2)-atan2(abs(y1-y2),abs(x1-x2))

    i += 1
    j += 2

# Creation of the doors 
# -> considering middle points of doors
# -> doors considered with normal angle and opposite angle (+pi)
# -> adding initial and end positions (door = (0,0))
doors_coordinates = [[0,0,0,0]]
for i in range(len(middle_points[0])):
    door_x = middle_points[0][i]
    door_y = middle_points[1][i]
    door_theta = middle_points_theta[0,i]
    doors_coordinates.append([door_x, door_y, door_theta, 0.3])
    doors_coordinates.append([door_x, door_y, door_theta+pi, 0.3])
doors_coordinates.append([0,0,0,0])


""" ---- Test all the possibilities of roads between doors ---- """
num_possibilities = len(doors_coordinates)
tab = np.zeros((num_possibilities, num_possibilities))

for i in range(num_possibilities):
  for j in range(num_possibilities):
    try :
      print(i,j)
      time, x, y, v, theta, a, delta = opc_problem_door_to_door(doors_coordinates[i], doors_coordinates[j])
    except RuntimeError:
      print("Non feasible solution")
      tab[i][j] = inf
    else:
      tab[i][j] = time


""" ---- Solve the TSP algorithm with calculated times ---- """
best_path, min_time = traveling_salesman_problem(doors_coordinates, tab) # door_coordinates = center of doors
print("Optimal path:", best_path)
print("Time using this path:", min_time)



""" ---- Solve OPC problem once again with best path to save the concatenated results ---- """
x_res = []
y_res = []
v_res = []
theta_res = []
for i in range(len(best_path)-1):
    time, x, y, v, theta, a, delta = opc_problem_door_to_door(doors_coordinates[best_path[i]], doors_coordinates[best_path[i+1]])
    x_res.extend(x)
    x_res.pop()
    y_res.extend(y)
    y_res.pop()
    v_res.extend(v)
    v_res.pop()
    theta_res.extend(theta)
    theta_res.pop()

x_res.append(x[-1]) # Generating the list of the concatenated x positions
y_res.append(y[-1]) # Generating the list of the concatenated y positions
v_res.append(v[-1]) # Generating the list of the concatenated v positions
theta_res.append(theta[-1]) # Generating the list of the concatenated theta positions



""" ---- Print the initial guess found solution ---- """
plt.figure() # 2D-Map of the problem
plt.plot(points[0,:],points[1,:],'Dr',ms=6, label="Doors")
for key, value in doors.items():
    plt.text(value[0][0]+0.1,value[0][1]+0.1,key)
plt.scatter(x_res, y_res,s=20, c=v_res, cmap='viridis')
plt.colorbar(label="Velocity [m/s]")
plt.title("Optimal Trajectory")
plt.xlabel("Position x")
plt.ylabel("Position y")#
plt.legend(loc="upper left")
plt.show()
print("Time using this path, calculated with OPC :", min_time)



""" ---- Saving the result in a separate file ---- """
def switch(door):
    switch_dict = {
        0: caseNone,
        1: case1,
        2: case1,
        3: case2,
        4: case2,
        5: case3,
        6: case3,
        7: case4,
        8: case4,
        9: caseNone,
    }
    case_func = switch_dict.get(door, caseNone)
    return case_func()

def case1():
    return {'A': doors['A']}

def case2():
    return {'B': doors['B']}

def case3():
    return {'C': doors['C']}

def case4():
    return {'D': doors['D']}

def caseNone():
    return None

order = {}
for door in best_path:
  door_dict = switch(door)
  if door_dict != None:
    for key, value in door_dict.items():
      order[key] = value

# filename = "Name of the test", to be modified if wanted
save_data(doors, order, x_res, y_res, v_res, theta_res, time, "Name of the test")