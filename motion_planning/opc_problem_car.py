from casadi import *
from math import pi
import numpy as np
from doors import middle_points, middle_points_theta, doors, points
# ---- post-processing ------
from pylab import plot, step, figure, legend, show, spy, title, xlabel, ylabel, annotate
import matplotlib.pyplot as plt


""" ---- Optimization problem ---- """
opti = casadi.Opti()

""" ---- Parameters ---- """
intervals = 20 # Number of control intervals per phase
n_phases = len(doors)+1 # Number of piecewise trajectories between doors and start/end point
N = n_phases*intervals

L = 0.16 # Length of the car
delta_min = -pi/4 # Minimum steering angle
delta_max = pi/4 # Maximum steering angle
a_min = -1 # Minimum acceleration
a_max = 1 # Maximum acceleration
v_max = 10 # Maximum velocity
ac_max = 1000 # Maximum centripetal acceleration
x_min = -20 # Min Boundary x of the track
y_min = -20 # Min Boundary y of the track
x_max = 20 # Max Boundary x of the track
y_max = 20 # Max Boundary y of the track

# Defining the intermediate states
X_waypoints = np.zeros(((4,len(doors))))
for i in range(len(X_waypoints[0])):
    X_waypoints[:,i] = [middle_points[0,i],
                       middle_points[1,i],
                       middle_points_theta[0,i],
                       0.1]

""" ---- State variables X ---- """
X = opti.variable(4,N+1)
x = X[0,:]                # x position
y = X[1,:]                # y position
theta = X[2,:]            # heading angle
v = X[3,:]                # velocity

""" ---- Control variables U ---- """
U = opti.variable(2,N+1)
delta = U[0,:]            # steering angle
a = U[1,:]                # acceleration

""" ---- Other variables ---- """
letter_list = []
t0 = opti.variable() # Time variable for the first path
time_list = [t0]

i = 1
for letter in doors:
   """
   Generates an optimization variable for each door, that
   determines the point between the gates through which the car
   should pass to ensure an optimal trajectory
   """
   locals()[letter] = opti.variable()  
   exec("letter_list.append({})".format(letter))
   exec("t{} = opti.variable()".format(i))
   exec("time_list.append(t{})".format(i))
   i += 1


""" ---- Cost function ---- """
opti.minimize(sum(time_list)) # Minimize the time to complete the trajectory

x1 = MX.sym('x1')
x2 = MX.sym('x2')
x3 = MX.sym('x3')
x4 = MX.sym('x4')
X_func = vertcat(x1,x2,x3,x4)

u1 = MX.sym('u1')
u2 = MX.sym('u2')
U_func = vertcat(u1,u2)

# ODE right handside
'''
xdot = v*cos(theta)
ydot = v*sin(theta)
thetadot = (v*sin(delta))/L
vdot = a
Xdot = vertcat(xdot,ydot,thetadot,vdot)
'''
x1dot = x4*cos(x3)
x2dot = x4*sin(x3)
x3dot = (x4*tan(u1))/L
x4dot = u2
Xdot = vertcat(x1dot,x2dot,x3dot,x4dot)


""" ---- ODE right handside function ---- """
f = Function('f', [X_func,U_func],[Xdot])


for h in range(n_phases):
   dt = time_list[h]/intervals # length of the control interval
   for j in range(intervals): # loop over control intervals
      # Runge-Kutta 4 integration
      k = j+(h*intervals)
      k1 = f(X[:,k],         U[:,k])
      k2 = f(X[:,k]+dt/2*k1, U[:,k])
      k3 = f(X[:,k]+dt/2*k2, U[:,k])
      k4 = f(X[:,k]+dt*k3,   U[:,k])
      x_next = X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
      opti.subject_to(X[:,k+1]==x_next) # close the gaps
   

""" ----- Boundary Conditions ----- """
opti.subject_to(x[0]==0) # Start x position
opti.subject_to(y[0]==0) # Start y position
opti.subject_to(v[0]==0) # Start v
#opti.subject_to(theta[0]==pi/2) # Start theta position

opti.subject_to(x[N]==0) # End x position
opti.subject_to(y[N]==0) # End y position
opti.subject_to(v[N]==0) # End v
#opti.subject_to(theta[N]==pi/2) # Endtheta position

for letter in letter_list:
   opti.subject_to(opti.bounded(0.2,letter,0.8))

i = 1
for key,value in doors.items():
   #opti.subject_to(X[:,int((i*N)/n_phases)]==X_waypoints[:,i-1]) # Intermediate states
   opti.subject_to(X[0,int((i*N)/n_phases)]==value[0][0]+letter_list[i-1]*(value[1][0]-value[0][0])) # x
   opti.subject_to(X[1,int((i*N)/n_phases)]==value[0][1]+letter_list[i-1]*(value[1][1]-value[0][1])) # y
   #opti.subject_to(X[3,int((i*N)/n_phases)]==X_waypoints[3,i-1]) # v
   i += 1


""" ----- Inequality Constraints -----"""
opti.subject_to(opti.bounded(delta_min,delta,delta_max)) # Limit on steering angle
opti.subject_to(opti.bounded(a_min,a,a_max))             # Limit on acceleration
opti.subject_to(v<=v_max)                                # Limit on velocity
opti.subject_to(v>=0)                                    # Velocity is greater or equal than zero
for time in time_list:
   opti.subject_to(time>=0)                              # Time must be positive

ac = lambda v,delta,L: (v**2*tan(delta))/(L)  # Centripetal acceleration definition
opti.subject_to(opti.bounded(-ac_max,ac(v,delta,L),ac_max))

""" ----- Environment Constraints ----- """
opti.subject_to(opti.bounded(x_min,x,x_max)) # Boundary x of the track
opti.subject_to(opti.bounded(y_min,y,y_max)) # Boundary y of the track

""" ----- Obstacles Constraints ----- """
xp = 1 # Position x of the circle
yp = 2 # Position y of the circle
R = 2 # Radius os the circle
#opti.subject_to((x-xp)**2+(y-yp)**2>=R**2) # Declaring the circle obstacle


""" ----- Initial Guess for State Variables ----- """
x_guess = np.zeros(N + 1)
y_guess = np.zeros(N + 1)
x_guess[0:int(N/(n_phases))] = np.linspace(0, X_waypoints[0][0], int(N/n_phases))  # Linear interpolation to first door
y_guess[0:int(N/(n_phases))] = np.linspace(0, X_waypoints[1][0], int(N/n_phases))  # Linear interpolation to first door
N_slice = 2

for i in range(len(doors)-1):
    x_guess[(N_slice-1)*int(N/(n_phases)):N_slice*int(N/(n_phases))] = np.linspace(X_waypoints[0][i], X_waypoints[0][i+1], int(N/n_phases))  # From door to door
    y_guess[(N_slice-1)*int(N/(n_phases)):N_slice*int(N/(n_phases))] = np.linspace(X_waypoints[1][i], X_waypoints[1][i+1], int(N/n_phases))  # From door to door
    i += 1
    N_slice += 1

x_guess[len(doors)*int(N/(n_phases)):N+1] = np.linspace(X_waypoints[0][i],0,int(N/n_phases)+1) # Return to starting point
y_guess[len(doors)*int(N/(n_phases)):N+1] = np.linspace(X_waypoints[1][i],0,int(N/n_phases)+1) # Return to starting point

theta_guess = np.zeros(N + 1)    # Assuming initial heading towards door
v_guess = np.ones(N + 1)         # Starting with moderate velocity

""" ----- Initial Guess for Other Variables ----- """
# Rough estimate of total time


""" ---- Setting initial guesses ---- """
opti.set_initial(X[:, :], np.vstack((x_guess, y_guess, theta_guess, v_guess)))
#opti.set_initial(T, T_guess)

""" ---- Solver ---- """
opti.solver('ipopt')
sol = opti.solve()


""" ---- Plotting ---- """
plt.figure()
#plt.plot(sol.value(x),label="x")
#plt.plot(sol.value(y),label="y")
plt.plot(sol.value(a),label="a")
plt.plot(sol.value(theta),label="theta")
plt.plot(sol.value(delta),label="delta")
plt.plot(sol.value(v),label="v")
plt.plot(ac(sol.value(v),sol.value(delta),L),label="ac")
plt.xlabel("N")
plt.ylabel("Values")
plt.legend(loc="upper right")


plt.figure()
plt.plot(0,0,'*b',ms=10, label="Start/End")
plt.plot(sol.value(x),sol.value(y),'g',ms=4,linewidth='0.5',label="Optimal trajectory")
plt.scatter(sol.value(x), sol.value(y),s=20, c=sol.value(v), cmap='viridis')

# Plotting obstacles
'''
circle1 = plt.Circle((xp, yp), R, color='blue')
ax = plt.gca()
ax.add_patch(circle1)
'''

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