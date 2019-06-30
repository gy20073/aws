#!/usr/bin/env python
# -*- coding: utf-8 -*-
from numpy import zeros, append, ones
import numpy as np
from math import cos, sin, pi
from scipy import stats
from dynamics_opt import f_6s
import rospy

class vehicle:

    def __init__(self, dt, maxDvxdt, random, seed, errorBound, F_side=0, linear=False):
        """
        vehicle model
        input: dt: timestep length
               linear: if true use linear model, else use nonlinear model
               random: boolean, randomize vehicle model
               seed: random seed, used as vehicle ID
               errBound: bound of random number
               F_side: Y direction side force
        state: 7 dof: pos & vel [X, Y, phi, v_x, v_y, r] + steering [d_f]
        """
        self.dt = dt
        self.F_side = F_side
        self.linear = linear
        self.setParameter(maxDvxdt)
        self.state = zeros(7)   # X(m), Y(m), phi(rad), v_x(m/s), v_y(m/s), r(rad/s), d_f (rad)
        if random:
            np.random.seed(seed)
            weights = np.random.uniform(-errorBound, errorBound, size=16)
            self.vhMdl = (weights[0:4] + 1) * self.vhMdl
            self.trMdl[0] = (weights[4:8] + 1) * self.trMdl[0]
            self.trMdl[1] = (weights[8:12] + 1) * self.trMdl[1]
            self.F_ext = (weights[12:14] + 1) * self.F_ext

    def setStart(self, X0, Y0, phi0, initial_speed=10):
        self.state = zeros(7)
        self.state[0] = X0
        self.state[1] = Y0
        self.state[2] = phi0
        self.state[3] = initial_speed

    def setState(self, state):
    	self.state = state

    def setParameter(self, maxDvxdt):
        # for vhMdl (based on LincolnMKZ vehicle model)
        a  = 1.20
        b  = 1.65
        m  = 1800.0
        Iz = 3270.0
        # for trMdl (now assuming front/rear tires are same)
        Bf = 10.0 * 0.72
        Br = 10.0 * 0.85
        C  = 1.9
        Df = 1.0
        Dr = 1.0
        E  = 0.97
        # for F_ext (now assuming no friction/resistance)
        a0 = 0.0
        Crr= 0.0
        # make params into tuple
        self.vhMdl = [a, b, m, Iz]
        self.trMdl = [[Bf, C, Df, E], [Br, C, Dr, E]]
        self.F_ext = [a0, Crr]
        # max steering and dvxdt
        self.maxSteeringRate = 650/180*pi
        self.maxSteering = 525.0/180*pi
        self.steeringRatio = 16         # steering angle / tire angle = 16
        self.maxDvxdt = maxDvxdt        # warning! tuning needed! (human factor, no need to rand)

    def simulate(self, action):
        """
        updates self.state
        input: action = [% longitudinal acceleration, % steering angle change] (-1 ~ 1) if not fixed speed
                        [% steering angle change] (-1 ~ 1)  if fixed speed
        output: void
        """
        self.state = self.simulateWithState(action, self.state)

    def simulateWithState(self, action, state):
        """
        given current state and action, calculate next step state
        input: action = [% longitudinal acceleration, % steering angle change] (-1 ~ 1)
               state: valid 7 dof state (pos & vel [X, Y, phi, v_x, v_y, r] + steering [d_f])
        output: nextState
        """
        nextState = zeros(7)
        action[0] = max(min(action[0], 1), -1)
        action[1] = max(min(action[1], 1), -1)

        dvxdt = self.maxDvxdt * action[0]
        steering = action[1] * self.maxSteeringRate * self.dt + state[6] * self.steeringRatio
        nextState[6] = max(min(steering, self.maxSteering), - self.maxSteering) / self.steeringRatio
        u = (nextState[6], dvxdt)
        nextState[:6] = f_6s(state[:6], u, self.vhMdl, self.trMdl, self.F_ext, self.dt, self.F_side, self.linear)
        
        return nextState