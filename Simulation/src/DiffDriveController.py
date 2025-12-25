#!/usr/bin/python

import numpy as np

class DiffDriveController():
    """
    Class used for controlling the robot linear and angular velocity
    """
    def __init__(self, max_speed, max_omega):
        # TODO for Student: Specify these parameters
        self.kp = 0.5  # Must be positive
        self.ka = 1  # Must be positive with ka - kp > 0
        self.kb = 0  # Must be negative
        self.error_tol = 0.05
        self.MAX_SPEED = max_speed
        self.MAX_OMEGA = max_omega
        
    def compute_vel(self, state, goal):
        """
        Function that computes the desired outputs given the state and goal
        Inputs:
        state - a numpy vector of size 3 by 1 with components (x,y,theta)
        goal - a numpy vector of size 2 by 1 specifying the location of the goal
        Outputs: a tuple with 3 elements
        v - a number specifying the forward speed (in m/s) of the robot (should 
            be no more than max_speed)
        omega - a number specifying the angular velocity (in rad/s) of the robot
            (should be no more than max_omega)
        done - a boolean value specifying if the robot has reached its goal (or
            is close enough
        """
        # YOUR CODE HERE
        x = state[0]
        y = state[1]
        theta = state[2]
        rho = np.linalg.norm([x-goal[0],y-goal[1]])
        alpha = -theta + np.arctan2(goal[1]-y,goal[0]-x)
        beta = -theta - alpha
        v = self.kp * rho
        omega = self.ka * alpha + self.kb * beta
        
        done = 0
        if rho<0.05:
            done = 1
        v = np.minimum(v,self.MAX_SPEED)
        omega = np.maximum(np.minimum(omega,self.MAX_OMEGA), -self.MAX_OMEGA)
        return (v,omega,done)
    