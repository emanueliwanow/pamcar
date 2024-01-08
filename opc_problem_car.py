from casadi import *
from math import pi

# Optimization problem
opti = casadi.Opti()

# Parameters
intervals = 20 # Number of control intervals per phase
n_phases = 2
N = n_phases*intervals
L = 0.16 # Length of the car
delta_min = -pi/4# Minimum steering angle
delta_max = pi/4 # Maximum steering angle
a_min = -0.1 # Minimum acceleration
a_max = 0.1 # Maximum acceleration
v_max = 10 # Maximum velocity
ac_max = 0.1 # Maximum centripetal acceleration
x_min = -10 # Min Boundary x of the track
y_min = -10 # Min Boundary y of the track
x_max = 10 # Max Boundary x of the track
y_max = 10 # Max Boundary y of the track
door1 = [2, 3]


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
nt = opti.variable()      # step in witch car passes the door

# Cost function
T = opti.variable()       # time to be minimized

# ODE right handside
xdot = v*cos(theta)

ydot = v*sin(theta)
thetadot = (v*sin(delta))/L
vdot = a
Xdot = vertcat(xdot,ydot,thetadot,vdot)

# Optimization problem
opti.minimize(T) # Minimize the time to complete the trajectory

# ODE right handside function
f = Function('f', [X,U],[Xdot])
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
opti.subject_to(x[N]==0) # End x position
opti.subject_to(y[N]==0) # End y position

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
opti.subject_to(x[nt]==door1[0])
opti.subject_to(y[nt]==door1[1])

# Initial guess

for i in range(N):
    opti.set_initial(X[0,:], 0)
    opti.set_initial(X[1,:], 0)
    opti.set_initial(X[2,:], 0)
    opti.set_initial(X[3,:], 0)
    opti.set_initial(U[0,:], 0)
    opti.set_initial(U[1,:], 0)

opti.set_initial(T,1)
opti.set_initial(nt,5)


# Solver
opti.solver('ipopt')


sol = opti.solve()




# ---- post-processing        ------
from pylab import plot, step, figure, legend, show, spy
"""
plot(sol.value(x),label="x")
plot(sol.value(y),label="y")
legend(loc="upper left")
"""
plot(sol.value(x),sol.value(y))

show()
