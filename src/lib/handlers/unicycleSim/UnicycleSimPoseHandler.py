#!/usr/bin/env python
"""
=======================================
UnicycleSimPose.py - 2D Pose provider for unicycleSimulator
=======================================
"""

import sys
from numpy import *

import lib.handlers.handlerTemplates as handlerTemplates

class UnicycleSimPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data):
        """
        Pose Handler for the simulated unicycle robot
        """
        try:
            self.simulator = shared_data['UnicycleSimulator']
        except KeyError:
            print "(POSE) ERROR: Unicycle Simulator doesn't seem to be initialized!"
            sys.exit(-1)

        self.last_pose = None

    def getPose(self, cached=False):
        """ Returns the most recent (x,y,theta) reading from unicycle simulator """

        if not cached or self.last_pose is None:
            # Get updated information
            self.last_pose = self.simulator.getPose()
            #print self.last_pose
        return self.last_pose


