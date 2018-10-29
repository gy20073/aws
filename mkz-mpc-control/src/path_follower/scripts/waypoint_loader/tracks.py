#!/usr/bin/env python
# -*- coding: utf-8 -*-
# define the track class
import scipy.io 
import scipy.optimize
import numpy as np
from math import sqrt, cos, sin, tan, pi
import os, rospkg
import rospy
class Tra:

    def __init__(self, name, horizon):
        """
        track class
        input: track.mat filename
               dt: timestep, must match the dt in vehicle class
               horizon: search area, must match the horizon in vehicle class
               deviation: for a parallel lane, indicates lateral deviation with base lane
               numPrePts: num of predicted waypoints at a certain timestep
               threshold: if farther away from lane than threshold, then terminate the episode
        state: x, y, psi for the watpoints
               size: length of the reference trajectory
               threshold: if deviation larger than threshold then report failure
               currentIndex: index of waypoint corresponding to current vehicle position
       
        fileName = name + ".mat"
        
        waypoints_mat = scipy.io.loadmat(fileName)[name] 
        """
        rospack = rospkg.RosPack()
        waypoints_mat = scipy.io.loadmat(os.path.join(rospack.get_path("path_follower"), "scripts", "waypoint_loader", name + ".mat"))[name]
        self.x     = waypoints_mat[0][:]
        self.y     = waypoints_mat[1][:]
        self.t     = waypoints_mat[2][:]
        self.size  = waypoints_mat.shape[1]
        self.currentIndex = 0
        self.horizon = horizon

    def searchClosestPt(self, pX, pY, standard_index):
        """
        search for the closest point with the designated point on the traj
        input:  pX, pY: accurate point position
                self.horizon: search in +/- 10* horizon
                standard_index: search around the standard index
        output: indexMin: closest traj point's index
                distMin:  distance from the closest traj point to the accurate point

        (one trick: only compare dist**2, save the time used to compute sqrt)
        (another trick: only consider waypoints with index in currentIndex +/- 10*horizon range)
        """
        indexMin = standard_index
        distSqrMin = (pX - self.x[standard_index])**2 + (pY - self.y[standard_index])**2
        for index in range(max(standard_index - self.horizon, 0), min(standard_index + self.horizon, self.size)):
            distSqr = (pX - self.x[index])**2 + (pY - self.y[index])**2
            if distSqr < distSqrMin:
                indexMin = index
                distSqrMin = distSqr
        return indexMin, sqrt(distSqrMin)

