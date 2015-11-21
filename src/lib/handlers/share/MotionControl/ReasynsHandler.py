#!/usr/bin/env python
"""
=======================================================
ReasynsHandler.py 
=======================================================

"""

from numpy import *
from __is_inside import *
import time, math, sys
import logging
import __ReasynsHelper as Reasyns
from collections import OrderedDict
from scipy.linalg import norm
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import project
import pymatlab

import lib.handlers.handlerTemplates as handlerTemplates
from lib.regions import Point

class ReasynsHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data, scalingPixelsToMeters):
        """
        Vector motion planning controller

        scalingPixelsToMeters (float): Scaling factor between RegionEditor map and Javier's map
        """
        self.numRobots              = []    # number of robots: number of agents in the specification, controlled by the local planner
        
        self.scalingPixelsToMeters = scalingPixelsToMeters
        self.updateWithPose     = {}
        self.forceUpdate        = False
        self.initial            = True
        self.previous_next_reg  = {}
        self.previous_curr_reg  = {}
        self.pose               = {}
        self.current_regVertices= {}
        self.next_regVertices   = {}
        self.system_print       = False       # for debugging. print on GUI ( a bunch of stuffs)
        self.Velocity           = None
        self.currentRegionPoly  = None
        self.nextRegionPoly     = None
        self.radius             = self.scalingPixelsToMeters*0.15
        self.trans_matrix       = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector
        self.timer              = time.time()

        self.pose = OrderedDict()
        
        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        if not self.numRobots == len(self.robotList):
            self.numRobots = len(self.robotList)
            # self.numRobots = 3
            print "WARNING: changing the number of robots to "+str(self.numRobots)+" to match the config file"
        for robot_name in self.robotList:
            self.updateWithPose[robot_name] = False
        
        self.drive_handler = {}
        self.pose_handler  = {}
        self.executor      = executor
        # Get references to handlers we'll need to communicate with
        for robot_name in self.robotList: # x must be a string
            self.drive_handler[robot_name] = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler, robot_name)
            self.drive_handler[robot_name].loco = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler, robot_name)
            self.pose_handler[robot_name] = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler, robot_name)

        # Get information about regions
        self.rfi = executor.proj.rfi
        self.coordmap_map2lab = executor.hsub.coordmap_map2lab
        self.coordmap_lab2map = executor.hsub.coordmap_lab2map 
        self.last_warning = 0

        self.map = {}
        for region in self.rfi.regions:
            self.map[region.name] = self.createRegionPolygon(region)

        # Generate bounding box for passing to the local planner
        # testSetOfRegions = self.rfi.regions    # TODO: hard-coding. decomposed region file doesn't store the boundary.
        testSetOfRegions = []
        testSetOfRegions.append([Point(0,0),Point(0,700),Point(700,0),Point(700,700)])
        for region in testSetOfRegions:
            regionPoints = self.getRegionVertices(region)
            print regionPoints 
            # if region.name == 'boundary':
            xv = [x for x,y in regionPoints]; yv = [y for x,y in regionPoints]
            limitsMap = [min(xv), max(xv), min(yv), max(yv)]
        # print 'Bounding box: '+str(limitsMap)
        self.limitsMap = limitsMap


        # setup matlab communication
        self.session = executor.proj.session
        # self.session = pymatlab.session_factory()

        Reasyns.initializeController(self.session, self.rfi.regions, self.scalingPixelsToMeters, limitsMap)

    def _stop(self):
        """
        Properly terminates all threads/computations in the handler. Leave no trace behind.
        """

        session.run('saveData();')
        logging.debug("Closing the connection...") 


    def gotoRegion(self, current_regIndices, next_regIndices, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.

        current_regAllIndices: dictionary of region indices
        next_regAllIndices: dictionary of region indices
        """

        departed            = {}
        arrived             = {}
        doUpdate            = {}

        logging.debug("current_regIndices:" + str(current_regIndices))
        logging.debug("next_regIndices: " + str(next_regIndices))

        if not self.previous_next_reg:
            for robot_name, current_reg in current_regIndices.iteritems():
                self.previous_next_reg[robot_name] = []
        if not self.previous_curr_reg:
            for robot_name, current_reg in current_regIndices.iteritems():
                self.previous_curr_reg[robot_name] = []

        for robot_name, current_reg in current_regIndices.iteritems():
            next_reg = next_regIndices[robot_name]

            self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])
            # print "pose: "+str(self.pose[robot_name])

            doUpdate[robot_name] = False 
            if not self.previous_curr_reg[robot_name] == current_reg or not self.previous_next_reg[robot_name] == next_reg:
                # Find our current configuration
                # self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])
                # print "pose: "+str(self.pose[robot_name])

                # Check if Vicon has cut out
                # TODO: this should probably go in posehandler?
                if math.isnan(self.pose[robot_name][2]):
                    print "WARNING: No Vicon data! Pausing."
                    self.drive_handler[robot_name].setVelocity(0, 0, 0)  # So let's stop
                    time.sleep(1)
                    #return False not leaving yet until all robots are checked

                # if self.system_print == True:
                print "Next Region is " +str(robot_name)+str(self.rfi.regions[next_reg].name)
                print "Current Region is " +str(robot_name)+str(self.rfi.regions[current_reg].name)
                logging.debug("Next Region is " + str(self.rfi.regions[next_reg].name))
                logging.debug("Current Region is " + str(self.rfi.regions[current_reg].name))

                if not current_reg == next_reg:
                    doUpdate[robot_name] = True
                    
                self.currentRegionPoly = self.map[self.rfi.regions[current_reg].name]
                regionPolyOld = Polygon.Polygon(self.currentRegionPoly)

                # NOTE: Information about region geometry can be found in self.rfi.regions:
                vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])).T
                self.current_regVertices[robot_name] = vertices

                vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])).T
                self.next_regVertices[robot_name] = vertices

                self.previous_next_reg[robot_name] = next_reg
                self.previous_curr_reg[robot_name] = current_reg

                """
                if current_reg == next_reg and not last:
                    logging.debug('stop moving, regions the same')
                    # No need to move!
                    self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                    continue
                    #return True not leaving until all robots are checked
                """

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        vx, vy, w = Reasyns.executeController(self.session, self.pose, self.rfi.regions, current_regIndices, next_regIndices, self.coordmap_lab2map, self.scalingPixelsToMeters, doUpdate)

        # save the data
        if (time.time() - self.timer) > 10:
            self.timer = time.time()
            self.session.run('saveData();')


        for idx, robot_name in enumerate(self.robotList):
            current_reg = current_regIndices[robot_name]

            logging.debug(robot_name + '-vx:' + str(vx[idx]) + '-vy:' + str(vy[idx]) + ' w:' + str(w[idx]))
            self.drive_handler[robot_name].setVelocity(vx[idx], vy[idx], w[idx], self.pose[robot_name][2])

            #logging.debug("pose:" + str(pose))
            departed[robot_name] = not is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.current_regVertices[robot_name])
            # Figure out whether we've reached the destination region
            # arrived[robot_name] = is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.next_regVertices[robot_name])
            arrived[robot_name] = False
            for ireg in range(len(self.rfi.regions)):
                if ireg != current_reg:
                    vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[ireg].getPoints()])).T
                    arrived[robot_name] |= is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices)

            if departed[robot_name] and (not arrived[robot_name]) and (time.time()-self.last_warning) > 0.5:
                #print "WARNING: Left current region but not in expected destination region"
                # Figure out what region we think we stumbled into
                for r in self.rfi.regions:
                    pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                    vertices = mat(pointArray).T
                    if is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices):
                        logging.info("I think I'm in " + r.name)
                        break
                self.last_warning = time.time()

        # if norm([v[idx] for idx in range(len(v))]) < 0.01:
        #     self.forceUpdate = True

        self.initial = False
        logging.debug("arrived:" + str(arrived))
        return (True in arrived.values()) #arrived[self.executor.hsub.getMainRobot().name]

    def createRegionPolygon(self,region,hole = None):
        if hole == None:
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = [x for x in region.getPoints(hole_id = hole)]
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(float(1)/self.scalingPixelsToMeters*pt[0],float(1)/self.scalingPixelsToMeters*pt[1]) for pt in pointArray]
        formedPolygon= Polygon.Polygon(regionPoints)
        return formedPolygon

    def getRegionVertices(self,region):
        if type(region).__name__ == 'Region':
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = region
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(float(1)/self.scalingPixelsToMeters*pt[0],float(1)/self.scalingPixelsToMeters*pt[1]) for pt in pointArray]
        return regionPoints
