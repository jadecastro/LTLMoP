#!/usr/bin/env python
"""
=======================================
basicSimPose.py - 2D Pose provider for basicSimulator
=======================================
"""

import sys
from numpy import *
import logging

import lib.handlers.handlerTemplates as handlerTemplates

class LocalPlannerPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data, robot_id, scalingPixelsToMeters, init_region, x=0, y=0):
        """
        Pose Handler for basic simulated robot

        robot_id (int): id of the robot, starting at 0.
        scalingPixelsToMeters (float): Scaling factor between RegionEditor map and Javier's map
        init_region (region): The name of the region where the simulated robot starts
        x (float): the initial x-value in map coordinates in lab units (default=0.)
        y (float): the initial y-value in map coordinates in lab units (default=0.)
        """

        self.scalingPixelsToMeters = scalingPixelsToMeters
        self.session = executor.proj.session
        rfi_original = executor.proj.loadRegionFile(decomposed=False)

        # Start in the center of the defined initial region
        init_region_obj = rfi_original.regions[rfi_original.indexOfRegionWithName(init_region)]
        center = init_region_obj.getCenter()
        cx = (center.x)/self.scalingPixelsToMeters + x
        cy = (center.y)/self.scalingPixelsToMeters + y
        
        self.session.run('vOut'+str(robot_id+1)+' = '+str(cx)+';')
        self.session.run('wOut'+str(robot_id+1)+' = '+str(cy)+';')
        #self.session.run('vOut'+str(robot_id+1)+' = 1;')
        #self.session.run('wOut'+str(robot_id+1)+' = 2;')

        self.robot_id = robot_id


    def getPose(self, cached=False):
        """ Returns the most recent (x,y) reading from matlab """

        # Get updated information
        logging.debug("  Pose: "+str(self.session.getvalue('vOut'+str(self.robot_id+1)))+', '+str(self.session.getvalue('wOut'+str(self.robot_id+1))))
        x = 1*self.scalingPixelsToMeters*self.session.getvalue('vOut'+str(self.robot_id+1))
        y = 1*self.scalingPixelsToMeters*self.session.getvalue('wOut'+str(self.robot_id+1))
        theta = 0

        return array([x, y, theta])


