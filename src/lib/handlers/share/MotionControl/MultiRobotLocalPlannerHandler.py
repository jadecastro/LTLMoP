#!/usr/bin/env python
"""
=======================================================
MultiRobotLocalPlannerHandler.py 
=======================================================

"""

from numpy import *
from __is_inside import *
import time, math
import logging
import __LocalPlannerHelper as LocalPlanner
from collections import OrderedDict

import lib.handlers.handlerTemplates as handlerTemplates

class MultiRobotLocalPlannerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        """
        Vector motion planning controller
        """

        self.system_print       = False       # for debugging. print on GUI ( a bunch of stuffs)
        self.previous_next_reg = None
        self.Velocity           = None
        self.currentRegionPoly  = None
        self.nextRegionPoly     = None

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]

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

        # setup matlab communication
        self.session = LocalPlanner.initializeLocalPlanner(self.rfi.regions, self.coordmap_map2lab)

    def gotoRegion(self, current_regIndices, next_regIndices, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.

        current_regAllIndices: dictionary of region indices
        next_regAllIndices: dictionary of region indices
        """

        current_regVertices = {}
        next_regVertices    = {}
        pose                = {}
        departed            = {}
        arrived             = {}
        goalArray           = {}
        goalPosition        = {}
        goalVelocity        = {}

        logging.debug("current_regIndices:" + str(current_regIndices))
        logging.debug("next_regIndices: " + str(next_regIndices))

        pose = OrderedDict()
        j = 0
        for robot_name, current_reg in current_regIndices.iteritems():
            next_reg = next_regIndices[robot_name]

            # Find our current configuration
            pose.update([(robot_name,self.pose_handler[robot_name].getPose())])

            # Check if Vicon has cut out
            # TODO: this should probably go in posehandler?
            if math.isnan(pose[robot_name][2]):
                print "WARNING: No Vicon data! Pausing."
                self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                time.sleep(1)
                #return False not leaving yet until all robots are checked

            ###This part will be run when the robot goes to a new region, otherwise, the original tree will be used.
            if not self.previous_next_reg == next_reg:

                if self.system_print == True:
                    print "Next Region is " + str(self.rfi.regions[next_reg].name)
                    print "Current Region is " + str(self.rfi.regions[current_reg].name)

                #set to zero velocity before tree is generated
                #self.drive_handler.setVelocity(0, 0)
                if last:
                    transFace = None
                else:
                    # Determine the mid points on the faces connecting to the next region (one goal point will be picked among all the mid points later in buildTree)
                    transFace   = None
                    goalArray[robot_name]   = [[],[]] # list of goal points (midpoints of transition faces)
                    face_normal = [[],[]] # normal of the trnasition faces
                    for i in range(len(self.rfi.transitions[current_reg][next_reg])):
                        pointArray_transface = [x for x in self.rfi.transitions[current_reg][next_reg][i]]
                        transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                        bundle_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                        bundle_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                        goalArray[robot_name] = hstack((goalArray[robot_name],vstack((bundle_x,bundle_y))))

                    if transFace is None:
                        print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.proj.rfi.regions[current_reg].name, self.proj.rfi.regions[next_reg].name)

            goalPosition[robot_name] = goalArray[robot_name]  #for now, assume there is only one face.
            goalVelocity[robot_name] = [0, 0]  # temporarily setting this to zero

            # NOTE: Information about region geometry can be found in self.rfi.regions:
            vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])).T
            current_regVertices[robot_name] = vertices

            vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])).T
            next_regVertices[robot_name] = vertices

            """
            if current_reg == next_reg and not last:
                logging.debug('stop moving, regions the same')
                # No need to move!
                self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                continue
                #return True not leaving until all robots are checked
            """

            j += 1

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        vx, vy = LocalPlanner.executeLocalPlanner(self.session, pose, goalPosition, goalVelocity)

        for idx, robot_name in enumerate(self.robotList):
            logging.debug(robot_name + '-vx:' + str(vx[idx]) + ' vy:' + str(vy[idx]))
            self.drive_handler[robot_name].setVelocity(vx[idx], vy[idx], pose[robot_name][2])

            #logging.debug("pose:" + str(pose))
            departed[robot_name] = not is_inside([pose[robot_name][0], pose[robot_name][1]], current_regVertices[robot_name])
            # Figure out whether we've reached the destination region
            arrived[robot_name] = is_inside([pose[robot_name][0], pose[robot_name][1]], next_regVertices[robot_name])

            if departed[robot_name] and (not arrived[robot_name]) and (time.time()-self.last_warning) > 0.5:
                #print "WARNING: Left current region but not in expected destination region"
                # Figure out what region we think we stumbled into
                for r in self.rfi.regions:
                    pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
                    vertices = mat(pointArray).T
                    if is_inside([pose[robot_name][0], pose[robot_name][1]], vertices):
                        logging.info("I think I'm in " + r.name)
                        break
                self.last_warning = time.time()

        #logging.debug("arrived:" + str(arrived))
        return (True in arrived.values()) #arrived[self.executor.hsub.getMainRobot().name]
