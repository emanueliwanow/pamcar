import numpy as np
from math import *




def declareCircleObstacle(opti,x,y,xp,yp,R):
    opti.subject_to((x-xp)**2+(y-yp)**2>=R**2)