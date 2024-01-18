from doors import middle_points, middle_points_theta, doors
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import Rbf

X_waypoints = np.zeros(((4,len(doors))))
for i in range(len(X_waypoints)):
    X_waypoints[:,i] = [middle_points[0,i],
                       middle_points[1,i],
                       middle_points_theta[0,i],
                       0.1]

intervals = 20 # Number of control intervals per phase
n_phases = len(doors)+1 # Number of piecewise trajectories between doors and start/end point
N = n_phases*intervals

# Initial guess for state variables
x_guess = np.zeros(N + 1)
y_guess = np.zeros(N + 1)
x_guess[0:int(N/(n_phases))] = np.linspace(0, X_waypoints[0][0], int(N/n_phases))  # Linear interpolation to first door
N_slice = 2

for i in range(len(doors)-1):
    x_guess[(N_slice-1)*int(N/(n_phases)):N_slice*int(N/(n_phases))] = np.linspace(X_waypoints[0][i], X_waypoints[0][i+1], int(N/n_phases))  # From door to door
    y_guess[(N_slice-1)*int(N/(n_phases)):N_slice*int(N/(n_phases))] = np.linspace(X_waypoints[1][i], X_waypoints[1][i+1], int(N/n_phases))  # From door to door
    i += 1
    N_slice += 1

x_guess[len(doors)*int(N/(n_phases)):N+1] = np.linspace(X_waypoints[0][i],0,int(N/n_phases)+1) # Return to starting point
y_guess[len(doors)*int(N/(n_phases)):N+1] = np.linspace(X_waypoints[1][i],0,int(N/n_phases)+1) # Return to starting point

theta_guess = np.zeros(N + 1)  # Assuming initial heading towards door
v_guess = np.ones(N + 1)  # Starting with moderate velocity
T_guess = 10  # Rough estimate of total time
