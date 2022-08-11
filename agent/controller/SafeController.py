import sys, os
from os.path import abspath, join, dirname
sys.path.insert(0, join(abspath(dirname(__file__)), '../../'))

import numpy as np
from abc import ABC, abstractmethod
from utils import GoalType
import cvxopt
from cvxopt import matrix
from cvxopt import solvers

class SafeController(ABC):
    def __init__(self, spec, model):

        self.model = model
        self.u_max = np.vstack(spec["u_max"])

    @abstractmethod
    def __call__(self,
        dt: float,
        processed_data: dict,
        u_ref: np.ndarray,
        goal: np.ndarray,
        goal_type: GoalType) -> np.ndarray:
        '''
            Driver procedure. Do not change
        '''
        pass

class UnsafeController(SafeController):
    def __call__(self,
        dt: float,
        processed_data: dict,
        u_ref: np.ndarray,
        goal: np.ndarray,
        goal_type: GoalType) -> np.ndarray:
        '''
            Driver procedure. Do not change
        '''
        return u_ref

class EnergyFunctionController(SafeController):
    """
    Energy function based safe controllers

    Attributes:
        _name
        _spec
        _model
    """
    def __init__(self, spec, model):
        self._spec = spec
        self._model = model
        self.d_min = spec['d_min']
        self.eta = spec['eta']
        self.k_v = spec['k_v']

    def phi_and_derivatives(self, dt, ce, co):
        """
        ce: cartesian position of ego # should be cartesian state?
        co: cartesian position of an obstacle # should be cartesian state?
        """
        n = np.shape(ce)[0]//2

        # raise NotImplementedError # TODO delete this line

        # TODO compute the following terms
        c = self._spec['c'] # c is some conservativeness factor, c > 0
        k = self.k_v # k is some scaling factor for dot(phi_0), k > 0
        d_min = self.d_min
        x_rel   = ce[0][0] - co[0][0]
        y_rel   = ce[1][0] - co[1][0]
        dot_x_rel   = ce[2][0] - co[2][0]
        dot_y_rel   = ce[3][0] - co[3][0]
        # print(f'x_rel = {x_rel}')
        # print(f'y_rel = {y_rel}')
        d   = np.sqrt(x_rel**2 + y_rel**2)
        dot_d       = (x_rel*dot_x_rel + y_rel * dot_y_rel)/d
        phi         = c + d_min**2 - d**2 - k * dot_d 
        p_phi_p_ce  = 0 # not used 
        p_phi_p_co  = 0 # not used 
        
        return phi, p_phi_p_ce, p_phi_p_co
    
    @abstractmethod
    def safe_control(self, u_ref, obs, dt, processed_data):
        """ Compute the safe control between ego and an obstacle.
        """
        pass
    
    def __call__(self,
        dt: float,
        processed_data: dict,
        u_ref: np.ndarray,
        goal: np.ndarray,
        goal_type: GoalType) -> np.ndarray:
        '''
            Driver procedure. Do not change
        '''
        us = []

        for obs in processed_data["obstacle_sensor_est"]:
            phi, u = self.safe_control(u_ref, obs, dt, processed_data)
            us.append((phi, u))
        
        sorted(us, key=lambda x:x[0], reverse=True) # larger phi first
        
        return us[0][1] # adopt the control that avoids the most dangerous collision.


class SafeSetController(EnergyFunctionController):
    def __init__(self, spec, model):
        super().__init__(spec, model)
        self._name = 'safe_set'

    def safe_control(self, u_ref, obs, dt, processed_data):
        """ Compute the safe control between ego and an obstacle.

        Safe set compute u by solving the following optimization:
        min || u - u_ref ||, 
        s.t.  dot_phi < - eta  or  phi < 0 (eta is the safety margin used in phi)

        => p_phi_p_xe.T * dot_xe        + p_phi_p_co.T * dot_co < - eta
        => p_phi_p_xe.T * (fx + fu * u) + p_phi_p_co.T * dot_co < - eta
        => p_phi_p_xe.T * fu * u < - eta - p_phi_p_xe.T * fx - p_phi_p_co.T * dot_co

        """
        ce = np.vstack([processed_data["cartesian_sensor_est"]["pos"], processed_data["cartesian_sensor_est"]["vel"]])  # ce: cartesian state of ego
        co = np.vstack([processed_data["obstacle_sensor_est"][obs]["rel_pos"], processed_data["obstacle_sensor_est"][obs]["rel_vel"]]) + ce  # co: cartesian state of the obstacle
        
        x =  np.vstack(processed_data["state_sensor_est"]["state"]) # [x,y,v_x,v_y] of the ego (robot) approximately equal to ce

        n = np.shape(ce)[0]//2

        # raise NotImplementedError # TODO delete this line

        # TODO compute the following terms
        phi, _, _ = self.phi_and_derivatives(dt, ce, co)

        # Compute the following terms for QP
        x_rel   = ce[0][0] - co[0][0]
        y_rel   = ce[1][0] - co[1][0]
        dot_x_rel   = ce[2][0] - co[2][0]
        dot_y_rel   = ce[3][0] - co[3][0]

        # CVXOPT QP solver in the form of min 1/2 * u^T * P * u + q^T * u, s.t. G * u <= h
        P = matrix(np.array([[2,0],[0,2]]), tc='d')
        q = matrix(np.array([-2*u_ref[0], -2*u_ref[1]]), tc='d')
        # G = matrix(np.array([[-(self.k_v*dot_x_rel)/(np.sqrt(x_rel**2 + y_rel**2)), -(self.k_v*dot_y_rel)/(np.sqrt(x_rel**2 + y_rel**2))], [1,0], [0,1], [-1,0], [0,-1]]), tc='d')
        # h = matrix(np.array([-self.eta + 2*(x_rel*dot_x_rel + y_rel*dot_y_rel) - self.k_v*(x_rel*dot_x_rel + y_rel*dot_y_rel)**2/(x_rel**2 + y_rel**2)**(3/2), 200, 200, 200, 200]), tc='d')
        G = matrix(np.array([[-(self.k_v*dot_x_rel)/(np.sqrt(x_rel**2 + y_rel**2)), -(self.k_v*dot_y_rel)/(np.sqrt(x_rel**2 + y_rel**2))]]), tc='d')
        h = matrix(np.array([-self.eta + 2*(x_rel*dot_x_rel + y_rel*dot_y_rel) - self.k_v*(x_rel*dot_x_rel + y_rel*dot_y_rel)**2/(x_rel**2 + y_rel**2)**(3/2)]), tc='d')
        sol = solvers.qp(P,q,G,h)
        u = sol['x']

        return phi, u

