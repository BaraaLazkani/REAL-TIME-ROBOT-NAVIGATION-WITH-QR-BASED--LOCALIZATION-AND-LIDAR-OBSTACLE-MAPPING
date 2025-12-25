#!/usr/bin/python
import numpy as np

#import pylab
import time
import math

class KalmanFilter:
    """
    Class to keep track of the estimate of the robots current state using the
    Kalman Filter
    """
    def __init__(self, markers):
        """
        Initialize all necessary components for Kalman Filter, using the
        markers (AprilTags) as the map
        Input: 
        markers - an N by 4 array loaded from the parameters, with each element
            consisting of (x,y,theta,id) where x,y gives the 2D position of a
            marker/AprilTag, theta gives its orientation, and id gives its
            unique id to identify which one you are seeing at any given
            moment
        """
        self.markers = markers
        self.last_time = 0.0  
        self.Q_t = np.eye(2)
        self.R_t = np.eye(3)
        # YOUR CODE HERE
        self.x_t =  np.array([[0.0], [0.0], [0.0]])

        self.P_t = np.eye(3)*1000.0
    def prediction(self, v, imu_meas):
        """
        Performs the prediction step on the state x_t and covariance P_t
        Inputs:
        v - a number representing in m/s the commanded speed of the robot
        imu_meas - a 5 by 1 numpy array consistening of the values
            (acc_x,acc_y,acc_z,omega,time), with the fourth of the values giving
            the gyroscope measurement for angular velocity (which you should
            use as ground truth) and time giving the current timestamp. Ignore
            the first three values (they are for the linear acceleration which
            we don't use)
        Outputs: a tuple with two elements
        predicted_state - a 3 by 1 numpy array of the predction of the state
        predicted_covariance - a 3 by 3 numpy array of the predction of the
            covariance
        """
        # YOUR CODE
        dt = imu_meas[4] - self.last_time
        self.last_time = imu_meas[4]
        th = self.x_t[2,0]
        self.x_t[0,0] += dt*v*np.cos(th)
        self.x_t[1,0] += dt*v*np.sin(th)
        self.x_t[2,0] += dt*imu_meas[3]
        
        #dt*np.array([v*np.cos(th),v*np.sin(th),-imu_meas[3]],dtype=float).T
        f_x = np.eye(3)*1.0 + dt* np.array([[0,0,-v*np.sin(th)],[0,0,v*np.cos(th)],[0,0,0]],dtype=float)
        f_n = dt * np.array([[np.cos(th),0],[np.sin(th),0],[0,1]],dtype=float)
        
        self.P_t = f_x @ self.P_t @ f_x.T + f_n @ self.Q_t @ f_n.T
        self.P_t = np.array(self.P_t, dtype=float)



    def tag_pos(self, tag_id):
        for i in range(len(self.markers)):
            if self.markers[i][3] == tag_id:
                return self.markers[i][:3]
        return None
    
    
    def step_filter(self, v, imu_meas, z_t):
        """
        Perform step in filter, called every iteration (on robot, at 60Hz)
        Inputs:
        v, imu_meas - descriptions in prediction. Will be None value if
            values are not available
        z_t - description in update. Will be None value if measurement is not
            available
        Outputs:
        x_t - current estimate of the state
        """
        # YOUR CODE HERE
        self.prediction(v, imu_meas)
        self.update(z_t)
        
        
        return self.x_t
    
    def update(self,z_t):
        """
        Performs the update step on the state x_t and covariance P_t
        Inputs:
        z_t - an array of length N with elements that are 4 by 1 numpy arrays.
            Each element has the same form as the markers, (x,y,theta,id), with
            x,y gives the 2D position of the measurement with respect to the
            robot, theta the orientation of the marker with respect to the
            robot, and the unique id of the marker, which you can find the
            corresponding marker from your map
        Outputs:
        predicted_state - a 3 by 1 numpy array of the updated state
        predicted_covariance - a 3 by 3 numpy array of the updated covariance
        """
        # YOUR CODE HERE
        H = np.eye(3)*1.0
        K = self.P_t @ H.T @ np.linalg.inv(H @ self.P_t @ (H.T) + self.R_t)
        
        if np.array(z_t).all():
            for z in z_t:
                tag_world_pose = self.tag_pos(z[3])
                tag_robot_pose = z[:3]
                R_wt = np.array([[np.cos(tag_world_pose[2]),-np.sin(tag_world_pose[2]),tag_world_pose[0]],
                        [np.sin(tag_world_pose[2]),np.cos(tag_world_pose[2]),tag_world_pose[1]],
                        [0,0,1]])
                R_ct = np.array([[np.cos(tag_robot_pose[2]),-np.sin(tag_robot_pose[2]),tag_robot_pose[0]],
                        [np.sin(tag_robot_pose[2]),np.cos(tag_robot_pose[2]),tag_robot_pose[1]],
                        [0,0,1]])
                R_bc = np.array([[1,0,0.021],[0,1,0.005],[0,0,1]])
                R = R_wt @ np.linalg.inv(R_ct) @ np.linalg.inv(R_bc) 
            
                zz = np.array([[R[0,2]],[R[1,2]],[np.arctan2(R[1,0],R[0,0])]])
                self.x_t += K @ ( zz - self.x_t)
                self.P_t = self.P_t - K @ H @ self.P_t
        