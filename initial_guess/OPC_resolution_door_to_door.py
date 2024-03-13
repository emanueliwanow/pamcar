from casadi import *
from math import pi
import numpy as np
from run_initial_guess import doors

def opc_problem_door_to_door(door1, door2):
  """ ---- Optimization problem initialisation ---- """
  problem = OPC_Problem(doors)
  problem.__init_door_to_door__(door1, door2)
  opti = problem.opti

  """ ---- Parameters ---- """
  problem.set_parameters()

  """ ---- Defining Optimization Variables ---- """
  problem.set_opti_variables_door_to_door()

  """ ---- Cost function ---- """
  problem.set_cost_function()

  """ ---- Defines right handside Function ---- """
  problem.creates_ODE_right_handside_function()

  """ ---- Direct Multiple Shooting ---- """
  problem.direct_multiple_shooting_RK4()

  """ ---- Constraints ---- """
  problem.set_equality_constraints_door_to_door()
  problem.set_inequality_constraints()
  problem.set_environment_constraints()

  """ ---- Initial Guess for State Variables ---- """
  x_guess, y_guess, theta_guess, v_guess, t_guess = problem.initial_guess_door_to_door()

  """ ---- Setting initial guesses ---- """
  problem.set_initial_guess_door_to_door(x_guess, y_guess, theta_guess, v_guess, t_guess)


  """ ---- DEBUG - Plot intermediate ---- """
  #def intermediate_plot(i):
  #  plt.figure(i)
  #  a = opti.debug.value(problem.x)
  #  b = opti.debug.value(problem.y)
  #  plt.plot(a,b,label="It."+str(i)+", Time = "+ str(opti.debug.value(problem.T)))
  #  plt.scatter(a, b,s=20, c=opti.debug.value(problem.v), cmap='viridis')
  #  plt.colorbar(label="Velocity [m/s]")
  #  plt.legend(loc="upper left")
  # opti.callback(intermediate_plot)


  """ ---- Solver ---- """
  opti.solver('ipopt')

  try:
    sol = opti.solve()
  except:
    print('Solver error')
    sol = opti.debug
    return inf, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
  else:
    return sol.value(problem.T), sol.value(problem.x), sol.value(problem.y), sol.value(problem.v), sol.value(
            problem.theta), sol.value(problem.a), sol.value(problem.delta)
