from casadi import *
from math import pi
import numpy as np

# Optimization problem
opti = casadi.Opti()

# Parameters
intervals = 100 # Number of control intervals per phase
n_phases = 2
N = n_phases*intervals
L = 0.16 # Length of the car
delta_min = -pi/4# Minimum steering angle
delta_max = pi/4 # Maximum steering angle
a_min = -0.1 # Minimum acceleration
a_max = 0.1 # Maximum acceleration
v_max = 1 # Maximum velocity
ac_max = 0.01 # Maximum centripetal acceleration
x_min = -10 # Min Boundary x of the track
y_min = -10 # Min Boundary y of the track
x_max = 10 # Max Boundary x of the track
y_max = 10 # Max Boundary y of the track

# Defining the intermediate state
door1 = [2, 3]
X_1 = [door1[0],door1[1],0,0.1]


# State variables
X = opti.variable(4,N+1)
x = X[0,:]                # x position
y = X[1,:]                # y position
theta = X[2,:]            # heading angle
v = X[3,:]                # velocity

# Control variables
U = opti.variable(2,N+1)
delta = U[0,:]            # steering angle
a = U[1,:]                # acceleration

# Cost function
T = opti.variable()       # time to be minimized
'''
# ODE right handside
xdot = v*cos(theta)
ydot = v*sin(theta)
thetadot = (v*sin(delta))/L
vdot = a
Xdot = vertcat(xdot,ydot,thetadot,vdot)
'''
# Optimization problem
opti.minimize(T) # Minimize the time to complete the trajectory

x1 = MX.sym('x1')
x2 = MX.sym('x2')
x3 = MX.sym('x3')
x4 = MX.sym('x4')

u1 = MX.sym('u1')
u2 = MX.sym('u2')

x1dot = x4*cos(x3)
x2dot = x4*sin(x3)
x3dot = (x4*sin(u1))/L
x4dot = u2

X_func = vertcat(x1,x2,x3,x4)
U_func = vertcat(u1,u2)
Xdot = vertcat(x1dot,x2dot,x3dot,x4dot)


# ODE right handside function
f = Function('f', [X_func,U_func],[Xdot])
M = 4
dt = T/N # length of the control interval
for k in range(N): # loop over control intervals
   # Runge-Kutta 4 integration
   k1 = f(X[:,k],         U[:,k])
   k2 = f(X[:,k]+dt/2*k1, U[:,k])
   k3 = f(X[:,k]+dt/2*k2, U[:,k])
   k4 = f(X[:,k]+dt*k3,   U[:,k])
   x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
   opti.subject_to(X[:,k+1]==x_next) # close the gaps
   

# Boundary Conditions
opti.subject_to(x[0]==0) # Start x position
opti.subject_to(y[0]==0) # Start y position
opti.subject_to(v[0]==0) # Start v
opti.subject_to(theta[0]==pi/2) # Start theta position


opti.subject_to(X[:,int(N/n_phases)]==X_1) # Intermediate state


opti.subject_to(x[N]==0) # End x position
opti.subject_to(y[N]==0) # End y position
opti.subject_to(v[N]==0) # End v

# Inequality Constraints
opti.subject_to(opti.bounded(delta_min,delta,delta_max)) # Limit on steering angle
opti.subject_to(opti.bounded(a_min,a,a_max)) # Limit on acceleration
opti.subject_to(v<=v_max) # Limit on velocity
opti.subject_to(v>=0)
opti.subject_to(T>=0) # Time must be positive

ac = lambda v,delta,L: (v**2*tan(delta))/(L*cos(delta)) # Centripetal acceleration definition
opti.subject_to(ac(v,delta,L)<=ac_max)

# Environment Constraints
opti.subject_to(opti.bounded(x_min,x,x_max)) # Boundary x of the track
opti.subject_to(opti.bounded(y_min,y,y_max)) # Boundary y of the track



# Initial guess for state variables
x_guess = np.zeros(N + 1)
x_guess[0:int(N/(n_phases))] = np.linspace(0, 2, int(N/n_phases))  # Linear interpolation to door
x_guess[int(N/(n_phases)):N+1] = np.linspace(2,0,int(N/n_phases)+1) # Return to starting point

y_guess = np.zeros(N + 1)
y_guess[0:int(N/(n_phases))] = np.linspace(0, 3, int(N/n_phases))  # Linear interpolation to door
y_guess[int(N/(n_phases)):N+1] = np.linspace(3,0,int(N/n_phases)+1) # Return to starting point

theta_guess = np.zeros(N + 1)  # Assuming initial heading towards door
v_guess = np.ones(N + 1)  # Starting with moderate velocity

# Initial guess for control variables
delta_guess = np.zeros(N + 1)  # Assuming mostly straight driving
"""
a_guess = np.linspace(0, 0.1, 10)  # Acceleration towards door
a_guess = np.append(a_guess, np.zeros(20))  # Constant speed around door
a_guess = np.append(a_guess, np.linspace(0, -0.1, 10))  # Deceleration back
a_guess = np.append(a_guess, 0)  # Add a single final element of 0
"""

# Initial guess for additional variables
T_guess = 10  # Rough estimate of total time


#print("x_guess shape:", x_guess)
#print("y_guess shape:", y_guess)
#print("theta_guess shape:", theta_guess)
#print("v_guess shape:", v_guess)
#print("delta_guess shape:", delta)
#print("a_guess shape:", a_guess)


# Now proceed with stacking and setting initial guesses
opti.set_initial(X[:, :], np.vstack((x_guess, y_guess, theta_guess, v_guess)))
#opti.set_initial(U[:, :], np.vstack((delta_guess, a_guess)))
opti.set_initial(T, T_guess)



# Solver
opti.solver('ipopt')
print("a",a)


sol = opti.solve()

# ---- post-processing        ------
from pylab import plot, step, figure, legend, show, spy

"""
plot(sol.value(x),label="x")
plot(sol.value(y),label="y")
legend(loc="upper left")

"""
figure()
plot(sol.value(x),sol.value(y))

figure()
plot(sol.value(v))
show()
