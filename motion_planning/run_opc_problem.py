from casadi import *
from math import pi
import numpy as np
from doors import doors
from opc_problem import OPC_Problem
# ---- post-processing ------
from pylab import plot, step, figure, legend, show, spy, title, xlabel, ylabel, annotate
import matplotlib.pyplot as plt


""" ---- Optimization problem ---- """
problem = OPC_Problem(doors)
opti = problem.opti

""" ----- Parameters -------------- """
problem.set_parameters()

""" ---- Defining Optimization Variables ---- """
problem.set_opti_variables()

""" ---- Cost function ---- """
problem.set_cost_function() 

""" ----- Defines right handside Function ----- """
problem.creates_ODE_right_handside_function()

"""------- Direct Multiple Shooting --------- """
problem.direct_multiple_shooting_RK4()

""" ----- Constraints ----- """
problem.set_equality_constraints()
problem.set_inequality_constraints()
problem.set_environment_constraints()

""" ----- Initial Guess for State Variables ----- """
middle_points, gates = problem.waypoints()
x_guess, y_guess = problem.simple_interpolated_initial_guess(middle_points)

""" ---- Setting initial guesses ---- """
problem.set_initial_guess(x_guess, y_guess)

"""Plot intermediate"""

def intermediate_plot(i):
   plt.figure(1)
   a = opti.debug.value(problem.x)
   b = opti.debug.value(problem.y)
   if i%10 == 0:
      plt.plot(a,b,label=str(i))
   

opti.callback(intermediate_plot)

""" ---- Solver ---- """
opti.solver('ipopt')

try:
   sol = opti.solve()
   SUCCESS = 1

except:
   print('Solver error')
   SUCCESS = 0


if SUCCESS:
   """ ---- Plotting ---- """
   plt.figure(2)
   #plt.plot(sol.value(x),label="x")
   #plt.plot(sol.value(y),label="y")
   plt.plot(sol.value(problem.a),label="a")
   plt.plot(sol.value(problem.theta),label="theta")
   plt.plot(sol.value(problem.delta),label="delta")
   plt.plot(sol.value(problem.v),label="v")
   plt.plot(problem.ac(sol.value(problem.v),sol.value(problem.delta),problem.L),label="ac")
   plt.xlabel("N")
   plt.ylabel("Values")
   plt.legend(loc="upper right")


   plt.figure(3)
   plt.plot(0,0,'*b',ms=10, label="Start/End")
   plt.plot(sol.value(problem.x),sol.value(problem.y),'g',ms=4,linewidth='0.5',label="Optimal trajectory")
   plt.scatter(sol.value(problem.x), sol.value(problem.y),s=20, c=sol.value(problem.v), cmap='viridis')

   # Plotting obstacles
   '''
   circle1 = plt.Circle((xp, yp), R, color='blue')
   ax = plt.gca()
   ax.add_patch(circle1)
   '''

   plt.plot(gates[0,:],gates[1,:],'Dr',ms=6, label="Doors")
   for key, value in doors.items():
      plt.text(value[0][0]+0.1,value[0][1]+0.1,key)
   #plt.plot(middle_points[0,:],middle_points[1,:],'x')
   plt.plot(x_guess,y_guess,linestyle='dotted', label="Initial guess")
   plt.colorbar(label="Velocity [m/s]")
   plt.title("Optimal Trajectory")
   plt.xlabel("Position x")
   plt.ylabel("Position y")
   plt.legend(loc="upper left")
   plt.axis('scaled')

   plt.show()
   
else:
   plt.figure(2)
   plt.plot(opti.debug.value(x),opti.debug.value(y),'g',ms=4,linewidth='0.5',label="Optimal trajectory")
   plt.scatter(opti.debug.value(x), opti.debug.value(y),s=20, c=opti.debug.value(v), cmap='viridis')
   plt.plot(points[0,:],points[1,:],'Dr',ms=6, label="Doors")
   for key, value in doors.items():
      plt.text(value[0][0]+0.1,value[0][1]+0.1,key)
   #plt.plot(middle_points[0,:],middle_points[1,:],'x')
   plt.plot(x_guess,y_guess,linestyle='dotted', label="Initial guess")
   plt.colorbar(label="Velocity [m/s]")
   plt.title("Optimal Trajectory")
   plt.xlabel("Position x")
   plt.ylabel("Position y")
   plt.legend(loc="upper left")
   plt.axis('scaled')
   plt.show()
   opti.debug.show_infeasibilities()
