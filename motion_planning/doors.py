import numpy as np
import matplotlib.pyplot as plt
from math import *
"""EDIT HERE THE DOORS YOU WANT
"""
# Parallel doors
doors = {'A':[(3,5),(3,4)],
         'B':[(8,5),(8,4)],
         'C':[(8,2),(8,1)],
         'D':[(3,2),(3,1)]}
"""

# Scenario 1
doors = {'A':[(2,6),(3,5)],
         'B':[(6,7),(6,6)],
         'C':[(10,5),(11,6)],
         'D':[(7,2),(7,1)]}


# Scenario 3
doors = {'A':[(3,3),(4,4)],
         'B':[(3,-3),(4,-4)],
         'C':[(-3,-3),(-4,-4)],
         'D':[(-3,3),(-4,4)]}
"""

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


