import os
os.path.join('../motion_planning', 'doors.py')
os.path.join('../motion_planning', 'opc_problem.py')
from doors import doors, middle_points, middle_points_theta
from opc_problem import *


""" ---- Changing format of doors (considering the middle point) and adding initial and end positions : door = (0,0) ---- """
doors_coordinates = [[0,0,0,0]]
for i in range(len(middle_points[0])):
    door_x = middle_points[0][i]
    door_y = middle_points[1][i]
    door_theta = middle_points_theta[0,i]
    doors_coordinates.append([door_x, door_y, 0.3, door_theta])
    doors_coordinates.append([door_x, door_y, 0.3, door_thet+pi])
doors_coordinates.append([0,0,0,0])

""" ---- Test all the possibilities of roads between doors with normal angle and opposite angle (+pi) ---- """
num_possibilities = len(doors_coordinates)
tab = np.zeros((num_possibilities, num_possibilities))

for i in range(num_possibilities):
  for j in range(num_possibilities):
    try :
      print(i,j)
      time, x, y, v, theta = opc_problem_door_to_door(doors_coordinates[i], doors_coordinates[j])
    except RuntimeError:
      print("Non feasible solution")
      tab[i][j] = inf
    else:
      tab[i][j] = time
print(tab)

""" ---- Solving the TSP algorithm with time already calculated with the OPC problem resolution ---- """
best_path, min_time = traveling_salesman_problem(doors_coordinates, tab) # door_coordinates = center of doors
print("Optimal path:", best_path)
print("Time using this path:", min_time)


""" ---- Solving OPC problem once again with best path to show the result ---- """
x_res = []
y_res = []
v_res = []
theta_res = []
for i in range(len(best_path)-1):
    time, x, y, v, theta = opc_problem_door_to_door(doors_coordinates[best_path[i]], doors_coordinates[best_path[i+1]])
    x_res.extend(x)
    y_res.extend(y)
    v_res.extend(v)
    theta_res.extend(theta)

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
print("Distance using this path, calculated with OPC :", min_time)




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

save_data(doors, order, x_res, y_res, v_res, theta_res, time, "Name of the test")