#!/usr/bin/env python
"""
=======================================
basicSimPose.py - 2D Pose provider for basicSimulator
=======================================
"""

import sys
from numpy import *

import lib.handlers.handlerTemplates as handlerTemplates

class LocalPlannerPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data, robot_id, scalingPixelsToMeters):
        """
        Pose Handler for basic simulated robot

        robot_id (int): id of the robot, starting at 0.
        scalingPixelsToMeters (float): Scaling factor between RegionEditor map and Javier's map
        """

        self.scalingPixelsToMeters = scalingPixelsToMeters
        self.robot_id = robot_id

        self.session = executor.proj.session

    def getPose(self, cached=False):
        """ Returns the most recent (x,y) reading from matlab """

        # Get updated information
        x = 1*self.scalingPixelsToMeters*self.session.getvalue('vOut'+str(self.robot_id+1))
        y = 1*self.scalingPixelsToMeters*self.session.getvalue('wOut'+str(self.robot_id+1))
        theta = 0

        return array([x, y, theta])


