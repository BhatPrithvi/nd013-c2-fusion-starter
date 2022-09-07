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

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dim_state = params.dim_state
        pass

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############
        F_matrix = np.matrix([[1, 0, 0, params.dt, 0, 0],
                             [0, 1, 0, 0, params.dt, 0],
                             [0, 0, 1, 0, 0, params.dt],
                             [0, 0, 0, 1, 0, 0],
                             [0, 0, 0, 0, 1, 0], 
                             [0, 0, 0, 0, 0, 1]])
        return F_matrix
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        q = params.q
        t2q = 1/3*(params.dt ** 3)*q
        tq = 1/2 *(params.dt ** 2)*q        
        Q_matrix = np.matrix([[t2q, 0, 0, tq, 0,0],
                            [0, t2q, 0, 0, tq, 0],
                            [0, 0, t2q, 0, 0, tq],
                            [tq, 0, 0, q, 0, 0],
                            [0, tq, 0, 0, q, 0],
                            [0, 0, tq, 0, 0, q]])
        return Q_matrix
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############
        x = self.F()*track.x # state prediction
        P = self.F()*track.P*self.F().transpose() + self.Q() # covariance prediction
        track.set_x(x)
        track.set_P(P)
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P*H.transpose()*np.linalg.inv(S) # Kalman gain
        x = track.x + K*self.gamma(track, meas)# state update
        I = np.identity(self.dim_state)
        P = (I - K*H) * track.P # covariance update
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
        Hx = meas.sensor.get_hx(track.x) # measurement matrix
        gamma = meas.z - Hx # residual
        ############
        return gamma
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############
        res_S = H*track.P*H.transpose() + meas.R # covariance of residual
        ############
        return res_S
        
        ############
        # END student code
        ############ 