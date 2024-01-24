from casadi import *
from math import pi
import numpy as np
""" ---- Post-processing ---- """
from pylab import plot, step, figure, legend, show, spy, title, xlabel, ylabel, annotate
import matplotlib.pyplot as plt

def opc_problem_door_to_door(door1, door2):
   """ ---- Optimization problem ---- """
   opti = casadi.Opti()

   """ ---- Parameters ---- """
   intervals = 100          # Number of control intervals per phase
   n_phases = 1             # Considering only one interval between two doors
   N = n_phases*intervals
   L = 0.16                 # Length of the car
   delta_min = -pi/4        # Minimum steering angle
   delta_max = pi/4         # Maximum steering angle
   a_min = -0.1             # Minimum acceleration
   a_max = 0.1              # Maximum acceleration
   v_max = 1                # Maximum velocity
   ac_max = 0.01            # Maximum centripetal acceleration
   x_min = -20              # Min Boundary x of the track
   y_min = -20              # Min Boundary y of the track
   x_max = 20               # Max Boundary x of the track
   y_max = 20               # Max Boundary y of the track

   """ ---- State variables ---- """
   X = opti.variable(4,N+1)
   x = X[0,:]               # x position
   y = X[1,:]               # y position
   theta = X[2,:]           # heading angle
   v = X[3,:]               # velocity

   """ ---- Control variables ---- """
   U = opti.variable(2,N+1)
   delta = U[0,:]           # steering angle
   a = U[1,:]               # acceleration

   """ ---- Cost function ---- """
   T = opti.variable()      # time to be minimized

   """ ---- Optimization problem ---- """
   opti.minimize(T)         # Minimize the time to complete the trajectory

   x1 = MX.sym('x1')
   x2 = MX.sym('x2')
   x3 = MX.sym('x3')
   x4 = MX.sym('x4')
   X_func = vertcat(x1,x2,x3,x4)

   u1 = MX.sym('u1')
   u2 = MX.sym('u2')
   U_func = vertcat(u1,u2)

   x1dot = x4*cos(x3)
   x2dot = x4*sin(x3)
   x3dot = (x4*sin(u1))/L
   x4dot = u2
   Xdot = vertcat(x1dot,x2dot,x3dot,x4dot)


   """ ---- ODE right handside function ---- """
   f = Function('f', [X_func,U_func],[Xdot])
   M = 4
   dt = T/N # length of the control interval
   for k in range(N): # Runge-Kutta 4 integration with loop over control intervals
      k1 = f(X[:,k],         U[:,k])
      k2 = f(X[:,k]+dt/2*k1, U[:,k])
      k3 = f(X[:,k]+dt/2*k2, U[:,k])
      k4 = f(X[:,k]+dt*k3,   U[:,k])
      x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4)
      opti.subject_to(X[:,k+1]==x_next) # close the gaps

   """ ---- Boundary Conditions (starting position at door1, ending position at door2) ---- """
   opti.subject_to(x[0]==door1[0])        # Start x position
   opti.subject_to(y[0]==door1[1])        # Start y position
   opti.subject_to(v[0]==door1[2])        # Start v speed
   opti.subject_to(theta[0]==door1[3])    # Start theta position

   opti.subject_to(x[N]==door2[0])        # End x position
   opti.subject_to(y[N]==door2[1])        # End y position
   opti.subject_to(v[N]==door2[2])        # End v speed
   opti.subject_to(theta[N]==door2[3])    # End v speed

   """ ---- Inequality Constraints ---- """
   opti.subject_to(opti.bounded(delta_min,delta,delta_max)) # Limit on steering angle
   opti.subject_to(opti.bounded(a_min,a,a_max))             # Limit on acceleration
   opti.subject_to(v<=v_max)                                # Limit on velocity
   opti.subject_to(v>=0)                                    # Velocity is greater or equal than zero
   opti.subject_to(T>=0)                                    # Time must be positive
   ac = lambda v,delta,L: (v**2*tan(delta))/(L*cos(delta))  # Centripetal acceleration definition
   opti.subject_to(ac(v,delta,L)<=ac_max)

   """ ---- Environment Constraints ---- """
   opti.subject_to(opti.bounded(x_min,x,x_max)) # Boundary x of the track
   opti.subject_to(opti.bounded(y_min,y,y_max)) # Boundary y of the track

   """ ---- Initial guess for state variables ---- """
   x_guess = np.linspace(door1[0], door2[0], N+1)   # Linear interpolation from door1 to door2
   y_guess = np.linspace(door1[1], door2[1], N+1)   # Linear interpolation from door1 to door2
   theta_guess = np.zeros(N + 1)                    # Assuming initial heading towards door
   v_guess = np.ones(N + 1)                         # Starting with moderate velocity

   """ ---- Initial guess for additional variables ---- """
   T_guess = 10  # Rough estimate of total time

   """ ---- Setting initial guesses ---- """
   opti.set_initial(X[:, :], np.vstack((x_guess, y_guess, theta_guess, v_guess)))
   opti.set_initial(T, T_guess)

   """ ---- Solver ---- """
   opti.solver('ipopt')
   sol = opti.solve()

   return sol.value(T), sol.value(x), sol.value(y), sol.value(v)