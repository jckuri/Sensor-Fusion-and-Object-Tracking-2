# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params 

def kalman_matrix(q1, q2, q3, q4):
    return np.matrix([[q1, 0,  0,  q2, 0,  0 ],
                      [0,  q1, 0,  0,  q2, 0 ],
                      [0,  0,  q1, 0,  0,  q2],
                      [q3, 0,  0,  q4, 0,  0 ],
                      [0,  q3, 0,  0,  q4, 0 ],
                      [0,  0,  q3, 0,  0,  q4]])


import random
np.random.seed(seed = 1234)
random.seed(1234)

class Filter:
    '''Kalman filter class'''
    def __init__(self):
    
        self.dim_state = params.dim_state # process model dimension
        self.dt = params.dt # time increment
        self.q = params.q # process noise variable for Kalman filter Q

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        dt = self.dt
        #return kalman_matrix(1, dt, 0, 1) # really good
        #return kalman_matrix(1, 0.1, 0, 1) # REALLY GOOD
        return kalman_matrix(1, 0.2, 0, 1) # GREAT
        #return kalman_matrix(1, 0.18, 0, 1)
        #return kalman_matrix(1, 0.15, 0, 1)
        #return kalman_matrix(1, 0, 0, 1)
        #return kalman_matrix(1, 0.1, 0, 1)
        
        return np.matrix([[1, 0, 0, dt, 0,  0 ],
                          [0, 1, 0, 0,  dt, 0 ],
                          [0, 0, 1, 0,  0,  dt],
                          [0, 0, 0, 1,  0,  0 ],
                          [0, 0, 0, 0,  1,  0 ],
                          [0, 0, 0, 0,  0,  1 ]])
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        
        #return kalman_matrix(1, 1, 1, 1) # really good
        #return kalman_matrix(0.5, 0.5, 0.5, 0.5)
        #return kalman_matrix(0.1, 0.1, 0.1, 0.1)
        #return kalman_matrix(0.01, 0.01, 0.01, 0.01) # REALLY GOOD
        #return kalman_matrix(0.01, 0.001, 0.001, 0.01) # MUCH BETTER
        return kalman_matrix(0.01, 0, 0, 0.01) # ALMOST EXCELLENT
        #return kalman_matrix(0.01, 0.001, 0.001, 0.01)
        #return kalman_matrix(0.005, 0.001, 0.001, 0.005)
        #return kalman_matrix(0.02, 0.02, 0.02, 0.02)
        #return kalman_matrix(0.005, 0.005, 0.005, 0.005)
        #return kalman_matrix(3, 1, 1, 3)
        
        q = self.q
        dt = self.dt
        q1 = ((dt**3)/3) * q 
        q2 = ((dt**2)/2) * q 
        q3 = dt * q 
        return np.matrix([[q1, 0,  0,  q2, 0,  0 ],
                          [0,  q1, 0,  0,  q2, 0 ],
                          [0,  0,  q1, 0,  0,  q2],
                          [q2, 0,  0,  q3, 0,  0 ],
                          [0,  q2, 0,  0,  q3, 0 ],
                          [0,  0,  q2, 0,  0,  q3]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        F = self.F()
        x = F * track.x # state prediction
        P = F * track.P * F.transpose() + self.Q() # covariance prediction
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        
        #print(f'Update track {track.id}: sensor.name={meas.sensor.name}, x.shape={track.x.shape}, x={track.x.transpose()}')
        x = track.x
        P = track.P
        H = meas.sensor.get_H(x) # measurement matrix
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H) # covariance of residual
        K = P * H.transpose() * np.linalg.inv(S) # Kalman gain
        x = x + K * gamma # state update
        I = np.identity(self.dim_state)
        P = (I - K * H) * P # covariance update
        track.set_x(x)
        track.set_P(P)
        
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############
        
        #x = track.x
        #H = meas.sensor.get_H(x) # measurement matrix
        #return meas.z - H * x # residual
        #print(f'GAMMA: meas.sensor.name={meas.sensor.name}, meas.z={meas.z.transpose()}')
        return meas.z - meas.sensor.get_hx(track.x)
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return H * track.P * H.transpose() + meas.R # covariance of residual
        
        ############
        # END student code
        ############ 
