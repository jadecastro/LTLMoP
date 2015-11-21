#!/usr/bin/env python
"""
=======================================================
vectorController.py - Vector Addition Motion Controller
=======================================================

Uses the vector field algorithm developed by Stephen R. Lindemann to calculate a global velocity vector to take the robot from the current region to the next region, through a specified exit face.
"""

import __vectorControllerHelper as vectorControllerHelper
from numpy import *
from __is_inside import *
import time, math
from collections import OrderedDict
import logging

import lib.handlers.handlerTemplates as handlerTemplates

class VectorControllerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        """
        Vector motion planning controller
        """

        self.numRobots = []    # number of robots: number of agents in the specification
        self.pose      = {}

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        if not self.numRobots == len(self.robotList):
            self.numRobots = len(self.robotList)
            # self.numRobots = 3
            print "WARNING: changing the number of robots to "+str(self.numRobots)+" to match the config file"
        
        # Get references to handlers we'll need to communicate with
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
        self.last_warning = 0

        self.pose = OrderedDict()

    def gotoRegion(self, current_regIndices, next_regIndices, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.
        """

        print current_regIndices
        for robot_name, current_reg in current_regIndices.iteritems():
            next_reg = next_regIndices[robot_name]
            logging.debug("Next Region is " + str(self.rfi.regions[next_reg].name))
            logging.debug("Current Region is " + str(self.rfi.regions[current_reg].name))

            if current_reg == next_reg and not last:
                # No need to move!
                self.drive_handler[robot_name].setVelocity(0, 0, 0)  # So let's stop
                return True

            # Find our current configuration
            self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])

            # Check if Vicon has cut out
            # TODO: this should probably go in posehandler?
            if math.isnan(self.pose[robot_name][2]):
                print "WARNING: No Vicon data! Pausing."
                self.drive_handler[robot_name].setVelocity(0, 0, 0)  # So let's stop
                time.sleep(1)
                return False

            # NOTE: Information about region geometry can be found in self.rfi.regions:
            pointArray = [x for x in self.rfi.regions[current_reg].getPoints()]
            pointArray = map(self.coordmap_map2lab, pointArray)
            vertices = mat(pointArray).T

            if last:
                transFaceIdx = None
            else:
                # Find a face to go through
                # TODO: Account for non-determinacy?
                # For now, let's just choose the largest face available, because we are probably using a big clunky robot
                # TODO: Why don't we just store this as the index?
                transFaceIdx = None
                max_magsq = 0
                print self.rfi.transitions
                for i, face in enumerate(self.rfi.regions[current_reg].getFaces()):
                    print face
                    if face not in self.rfi.transitions[current_reg][next_reg]:
                        continue

                    tf_pta, tf_ptb = face
                    tf_vector = tf_ptb - tf_pta
                    magsq = (tf_vector.x)**2 + (tf_vector.y)**2
                    if magsq > max_magsq:
                        transFaceIdx = i
                        max_magsq = magsq

                logging.debug('transFaceIdx:' + str(transFaceIdx))
                if transFaceIdx is None:
                    print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.rfi.regions[current_reg].name, self.rfi.regions[next_reg].name)

    		# Run algorithm to find a velocity vector (global frame) to take the robot to the next region
            logging.debug(robot_name + 'xpose:' + str(self.pose[robot_name][0]) + ' ypose:' + str(self.pose[robot_name][1]) + ' vertices:' + str(vertices) + ' transFaceIdx:' + str(transFaceIdx))
            V = vectorControllerHelper.getController([self.pose[robot_name][0], self.pose[robot_name][1]], vertices, transFaceIdx)
            logging.debug(robot_name + 'vx:' + str(V[0]) + ' vy:' + str(V[1]))

            # Pass this desired velocity on to the drive handler
            self.drive_handler[robot_name].setVelocity(V[0], V[1], 0, self.pose[robot_name][2])

            departed = not is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices)
            # Figure out whether we've reached the destination region
            arrived = is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices)

            if departed and (not arrived) and (time.time()-self.last_warning) > 0.5:
                #print "WARNING: Left current region but not in expected destination region"
                # Figure out what region we think we stumbled into
                for r in self.rfi.regions:
                    pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                    vertices = mat(pointArray).T

                    if is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices):
                        #print "I think I'm in " + r.name
                        #print self.pose[robot_name]
                        break
                self.last_warning = time.time()

        return arrived
