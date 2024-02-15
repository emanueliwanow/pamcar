import numpy as np
from math import*
from casadi import *

class OPC_Problem():
    """
    Class for solving multi-target optimization problem for 2S scenario with car

    Initialization:
        opti: Defines an optimization problem from Casadi package
        doors: Sets dictionary containing the goals through which the car must pass
        N: n_phases x intervals , which is the total number of discretization points for the problem

    Functions:
        set_opti_variables()
        waypoints()
        simple_interpolated_initial_guess()
    """
    def __init__(self, doors):
        self.opti = casadi.Opti()
        self.doors = doors
        self.intervals = 20               # Number of control intervals per phase
        self.n_phases = len(self.doors)+1 # Number of piecewise trajectories between doors and start/end point
        self.N = self.n_phases*self.intervals

    def set_parameters(self):
        """
        Sets parameters for model, equality, inequality constraints
        """
        self.L = 1.6                # Length of the car
        self.delta_min = -pi/4      # Minimum steering angle
        self.delta_max = pi/4       # Maximum steering angle
        self.a_min = -5             # Minimum acceleration
        self.a_max = 5              # Maximum acceleration
        self.v_max = 10             # Maximum velocity
        self.ac_max = 2             # Maximum centripetal acceleration
        self.x_min = -20            # Min Boundary x of the track
        self.y_min = -20            # Min Boundary y of the track
        self.x_max = 20             # Max Boundary x of the track
        self.y_max = 20             # Max Boundary y of the track
        self.theta_start = pi       # Desired Start for theta (in Rad)
        self.theta_end = -pi         # Desired End for theta (in Rad)

    def set_opti_variables(self):
        """
        Sets opti variables for altered Kinematic Bicycle Model
        """

        """ ---- State variables X ---- """
        self.X = self.opti.variable(4,self.N+1)
        self.x = self.X[0,:]                # x position
        self.y = self.X[1,:]                # y position
        self.theta = self.X[2,:]            # Heading angle
        self.v = self.X[3,:]                # Velocity

        """ ---- Control variables U ---- """
        self.U = self.opti.variable(2,self.N+1)
        self.delta = self.U[0,:]            # Steering angle
        self.a = self.U[1,:]                # Acceleration

        """ ---- Other variables ---- """
        self.letter_list = []
        t0 = self.opti.variable()           # Time variable for the first path
        self.time_list = [t0]

        i = 1
        for letter in self.doors:
            """
            Generates an optimization variable for each door, that
            determines the point between the gates through which the car
            should pass to ensure an optimal trajectory
            """
            locals()[letter] = self.opti.variable()  
            exec("self.letter_list.append({})".format(letter))
            exec("t{} = self.opti.variable()".format(i))
            exec("self.time_list.append(t{})".format(i))
            i += 1

    def set_cost_function(self):
        """
        Sets cost function minimizing the time to complete the whole trajectory
        """
        self.opti.minimize(sum(self.time_list)) 


    def creates_ODE_right_handside_function(self):
        """
        xdot = v*cos(theta)
        ydot = v*sin(theta)
        thetadot = (v*sin(delta))/L
        vdot = a
        Xdot = vertcat(xdot,ydot,thetadot,vdot)
        """
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
        x3dot = (x4*tan(u1))/self.L
        x4dot = u2
        Xdot = vertcat(x1dot,x2dot,x3dot,x4dot)

        self.f = Function('f', [X_func,U_func],[Xdot])

    def direct_multiple_shooting_RK4(self):
        """
        Direct Multiple SHooting with single shooting for each piece of the Runge Kutta 4
        """

        for h in range(self.n_phases):
            dt = self.time_list[h]/self.intervals               # length of the control interval
            for j in range(self.intervals):                     # loop over control intervals
                # Runge-Kutta 4 integration
                k = j+(h*self.intervals)
                k1 = self.f(self.X[:,k],         self.U[:,k])
                k2 = self.f(self.X[:,k]+dt/2*k1, self.U[:,k])
                k3 = self.f(self.X[:,k]+dt/2*k2, self.U[:,k])
                k4 = self.f(self.X[:,k]+dt*k3,   self.U[:,k])
                x_next = self.X[:,k] + dt/6*(k1+2*k2+2*k3+k4) 
                self.opti.subject_to(self.X[:,k+1]==x_next)     # close the gaps

    def set_equality_constraints(self):
        """
        Sets boundary conditions (initial and final states) and constraint that car must pass between gates of each door
        """
        self.opti.subject_to(self.x[0]==0)                             # Start x position
        self.opti.subject_to(self.y[0]==0)                             # Start y position
        self.opti.subject_to(self.v[0]==0)                             # Start v
        theta_start_cos = round(cos(self.theta_start),2)
        self.opti.subject_to(cos(self.theta[0])==theta_start_cos)      # Start theta position
        theta_start_sin = round(sin(self.theta_start),2)
        self.opti.subject_to(sin(self.theta[0])==theta_start_sin)      # Start theta position

        self.opti.subject_to(self.x[self.N]==0)                        # End x position
        self.opti.subject_to(self.y[self.N]==0)                        # End y position
        self.opti.subject_to(self.v[self.N]==0)                        # End v
        theta_end_cos = round(cos(self.theta_end),2)
        self.opti.subject_to(cos(self.theta[self.N])==theta_end_cos)   # End theta position
        theta_end_sin = round(sin(self.theta_end),2)
        self.opti.subject_to(sin(self.theta[self.N])==theta_end_sin)   # End theta position


        for letter in self.letter_list:
            self.opti.subject_to(self.opti.bounded(0.2,letter,0.8))

        i = 1
        for key,value in self.doors.items():
            self.opti.subject_to(self.X[0,int((i*self.N)/self.n_phases)]==value[0][0]+self.letter_list[i-1]*(value[1][0]-value[0][0])) # x
            self.opti.subject_to(self.X[1,int((i*self.N)/self.n_phases)]==value[0][1]+self.letter_list[i-1]*(value[1][1]-value[0][1])) # y
            i += 1

    def set_inequality_constraints(self):
        """ ----- Inequality Constraints -----"""
        self.opti.subject_to(self.opti.bounded(self.delta_min,self.delta,self.delta_max)) # Limit on steering angle
        self.opti.subject_to(self.opti.bounded(self.a_min,self.a,self.a_max))             # Limit on acceleration
        self.opti.subject_to(self.opti.bounded(0,self.v,self.v_max))                      # Limit on velocity
        for time in self.time_list:
            self.opti.subject_to(time>=0)                                                 # Time must be positive

        self.ac = lambda v,delta,L: (v**2*tan(delta))/(L)                                      # Centripetal acceleration definition
        self.opti.subject_to(self.opti.bounded(-self.ac_max,self.ac(self.v,self.delta,self.L),self.ac_max))

    def set_environment_constraints(self):
        """ ----- Environment Constraints ----- """
        self.opti.subject_to(self.opti.bounded(self.x_min,self.x,self.x_max)) # Boundary x of the track
        self.opti.subject_to(self.opti.bounded(self.y_min,self.y,self.y_max)) # Boundary y of the track

        """ ----- Obstacles Constraints ----- """
        xp = 1 # Position x of the circle
        yp = 2 # Position y of the circle
        R = 2 # Radius os the circle
        #self.opti.subject_to((self.x-xp)**2+(self.y-yp)**2>=R**2) # Declaring the circle obstacle

    def waypoints(self):
        """
        Calculates waypoints for initial guess.

        Returns:
            middle_points: Array with x,y coordinates of the middle points for each door
            gates: Array with x,y coordinates of each gate of each door

        """
        # Creating arrays with zeros
        middle_points = np.zeros((2,len(self.doors)))     # Middle points for each door
        gates = np.zeros((2,2*len(self.doors)))           # Saves position of gates for each door

        i = 0
        j = 0
        for key, value in self.doors.items():
            # Getting x,y coordinates from doors
            x1 = value[0][0]
            y1 = value[0][1]
            x2 = value[1][0]
            y2 = value[1][1]

            # Getting middle point inside door
            middle_x = (x1+x2)/2
            middle_y = (y1+y2)/2

            # Putting coordinates in array for visualization
            gates[:,j] = [x1,y1]
            gates[:,j+1] = [x2,y2]
            middle_points[:,i] = [middle_x,middle_y]

            i += 1
            j += 2
        
        return middle_points, gates


    def simple_interpolated_initial_guess(self,middle_points):
        """
        Calculates initial guess through interpolation between middle points of each door.

        Returns:
            x_guess: Array with N+1 values for initial guess of x
            y_guess: Array with N+1 values for initial guess of y

        """
        # Initial Guess
        x_guess = np.zeros(self.N + 1)
        y_guess = np.zeros(self.N + 1)
        x_guess[0:int(self.N/(self.n_phases))] = np.linspace(0, middle_points[0][0], int(self.N/self.n_phases))  # Linear interpolation to first door
        y_guess[0:int(self.N/(self.n_phases))] = np.linspace(0, middle_points[1][0], int(self.N/self.n_phases))  # Linear interpolation to first door
        N_slice = 2

        for i in range(self.n_phases-2):
            x_guess[(N_slice-1)*int(self.N/(self.n_phases)):N_slice*int(self.N/(self.n_phases))] = np.linspace(middle_points[0][i], middle_points[0][i+1], int(self.N/self.n_phases))  # From door to door
            y_guess[(N_slice-1)*int(self.N/(self.n_phases)):N_slice*int(self.N/(self.n_phases))] = np.linspace(middle_points[1][i], middle_points[1][i+1], int(self.N/self.n_phases))  # From door to door
            i += 1
            N_slice += 1

        x_guess[(self.n_phases-1)*int(self.N/(self.n_phases)):self.N+1] = np.linspace(middle_points[0][i],0,int(self.N/self.n_phases)+1) # Return to starting point
        y_guess[(self.n_phases-1)*int(self.N/(self.n_phases)):self.N+1] = np.linspace(middle_points[1][i],0,int(self.N/self.n_phases)+1) # Return to starting point

        return x_guess, y_guess

    def set_initial_guess(self, x_guess, y_guess):
        self.opti.set_initial(self.X[0:2, :], np.vstack((x_guess, y_guess)))
        self.opti.set_initial(self.theta[0],self.theta_start)
        self.opti.set_initial(self.theta[self.N],self.theta_end)