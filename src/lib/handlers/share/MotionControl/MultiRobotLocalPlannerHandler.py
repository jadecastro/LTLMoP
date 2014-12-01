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
from scipy.linalg import norm
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes

import lib.handlers.handlerTemplates as handlerTemplates

class MultiRobotLocalPlannerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data):
        """
        Vector motion planning controller
        """

        self.goal               = {}
        self.previous_next_reg  = {}
        self.pose               = {}
        self.goalPosition = {}
        self.goalVelocity = {}
        self.current_regVertices= {}
        self.next_regVertices   = {}
        self.system_print       = False       # for debugging. print on GUI ( a bunch of stuffs)
        self.Velocity           = None
        self.currentRegionPoly  = None
        self.nextRegionPoly     = None
        self.radius             = 0.15
        self.trans_matrix       = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector

        self.pose = OrderedDict()
        self.goalPositionList = OrderedDict()
        self.goalVelocityList = OrderedDict()

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        for robot_name in self.robotList:
            self.goalPosition[robot_name] = []
            self.goalVelocity[robot_name] = []
            self.goalPositionList[robot_name] = []
            self.goalVelocityList[robot_name] = []

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

        # Generate polygon for regions in the map
        self.map = {}
        for region in self.rfi.regions:
            self.map[region.name] = self.createRegionPolygon(region)
            # for n in range(len(region.holeList)): # no of holes
            #     self.map[region.name] -= self.createRegionPolygon(region,n)

        # setup matlab communication
        self.session = LocalPlanner.initializeLocalPlanner(self.rfi.regions, self.coordmap_map2lab)

    def gotoRegion(self, current_regIndices, next_regIndices, last=False):
        """
        If ``last`` is True, we will move to the center of the destination region.

        Returns ``True`` if we've reached the destination region.

        current_regAllIndices: dictionary of region indices
        next_regAllIndices: dictionary of region indices
        """

        departed            = {}
        arrived             = {}
        goalArray           = {}
        face_normal         = {}

        logging.debug("current_regIndices:" + str(current_regIndices))
        logging.debug("next_regIndices: " + str(next_regIndices))

        if not self.previous_next_reg:
            for robot_name, current_reg in current_regIndices.iteritems():
                self.previous_next_reg[robot_name] = []

        for robot_name, current_reg in current_regIndices.iteritems():
            next_reg = next_regIndices[robot_name]

            doUpdate = False            
            if not self.previous_next_reg[robot_name] == next_reg:
                # Find our current configuration
                self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])

                # Check if Vicon has cut out
                # TODO: this should probably go in posehandler?
                if math.isnan(self.pose[robot_name][2]):
                    print "WARNING: No Vicon data! Pausing."
                    self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                    time.sleep(1)
                    #return False not leaving yet until all robots are checked

                if self.system_print == True:
                    print "Next Region is " + str(self.rfi.regions[next_reg].name)
                    print "Current Region is " + str(self.rfi.regions[current_reg].name)
                logging.debug("Next Region is " + str(self.rfi.regions[next_reg].name))
                logging.debug("Current Region is " + str(self.rfi.regions[current_reg].name))

                self.currentRegionPoly = self.map[self.rfi.regions[current_reg].name]
                regionPolyOld = Polygon.Polygon(self.currentRegionPoly)

                if not current_reg == next_reg:
                    if last:
                        transFace = None
                    else:
                        # Determine the mid points on the faces connecting to the next region (one goal point will be picked among all the mid points later in buildTree)
                        transFace   = None
                        goalArray[robot_name]   = [[],[]] # list of goal points (midpoints of transition faces)
                        face_normal[robot_name] = [[],[]] # normal of the trnasition faces
                        for i in range(len(self.rfi.transitions[current_reg][next_reg])):
                            # if next_reg != current_reg:
                            logging.debug("  transition: " + str(self.rfi.transitions[current_reg][next_reg][i]))
                            pointArray_transface = [x for x in self.rfi.transitions[current_reg][next_reg][i]]
                            transFace = asarray(map(self.coordmap_map2lab,pointArray_transface))
                            bundle_x = (transFace[0,0] +transFace[1,0])/2    #mid-point coordinate x
                            bundle_y = (transFace[0,1] +transFace[1,1])/2    #mid-point coordinate y
                            goalArray[robot_name] = hstack((goalArray[robot_name],vstack((bundle_x,bundle_y))))

                            #find the normal vector to the face
                            face          = transFace[0,:] - transFace[1,:]
                            distance_face = norm(face)
                            normal        = face/distance_face * self.trans_matrix
                            face_normal[robot_name]   = hstack((face_normal[robot_name],vstack((normal[0,0],normal[0,1]))))
                            
                        if transFace is None:
                            print "ERROR: Unable to find transition face between regions %s and %s.  Please check the decomposition (try viewing projectname_decomposed.regions in RegionEditor or a text editor)." % (self.proj.rfi.regions[current_reg].name, self.proj.rfi.regions[next_reg].name)

                    goalArrayNew = mat(goalArray[robot_name])
                    face_normalNew = mat(face_normal[robot_name])
                    doUpdate = True

                    i = 0
                    goal = 2*[[]]
                    while i < goalArrayNew.shape[1]:
                        goal1 = goalArrayNew[:,i]-face_normalNew[:,i]*3*self.radius    ##original 2*self.radius
                        goal2 = goalArrayNew[:,i]+face_normalNew[:,i]*3*self.radius    ##original 2*self.radius
                        if regionPolyOld.isInside(goal1[0], goal1[1]):
                            goal[0] = goal2
                            goal[1] = goal1
                        else:
                            goal[0] = goal1
                            goal[1] = goal2
                        i += 1

                    self.goalPositionList[robot_name] = []
                    self.goalVelocityList[robot_name] = []
                    for i in range(len(goal)):
                        self.goalPositionList[robot_name].append(goal[i])  #for now, assume there is only one face.
                        self.goalVelocityList[robot_name].append([0, 0])  # temporarily setting this to zero

                else:
                    goal = self.goal[robot_name]
                
                self.goal[robot_name] = goal

                # NOTE: Information about region geometry can be found in self.rfi.regions:
                vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[current_reg].getPoints()])).T
                self.current_regVertices[robot_name] = vertices

                vertices = mat(map(self.coordmap_map2lab, [x for x in self.rfi.regions[next_reg].getPoints()])).T
                self.next_regVertices[robot_name] = vertices

                self.previous_next_reg[robot_name] = next_reg

                """
                if current_reg == next_reg and not last:
                    logging.debug('stop moving, regions the same')
                    # No need to move!
                    self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                    continue
                    #return True not leaving until all robots are checked
                """

            # if len(self.goalPosition[robot_name]) > 0:
            #     print "norm: ", norm(mat(self.pose[robot_name][:2]) - self.goalPosition[robot_name])
            #     print "diff: ", mat(self.pose[robot_name][:2]).T - self.goalPosition[robot_name]
            # print "pose: ", self.pose[robot_name][:2]
            # print self.goalPosition[robot_name]
            # print all(self.goalPosition[robot_name])
            # print self.goalPosition[robot_name]
            # print "goal: ", self.goal[robot_name]
            # print "goalPositionList: ", self.goalPositionList[robot_name]
            if len(self.goalPosition[robot_name]) > 0:
                if norm(mat(self.pose[robot_name][:2]).T - self.goalPosition[robot_name]) > 1.5*self.radius or len(self.goalPositionList[robot_name]) == 0:
                    pass
                else:
                    doUpdate = True
            else: 
                doUpdate = True
            if doUpdate:
                print "updating goal!!"
                self.goalPosition[robot_name] = self.goalPositionList[robot_name].pop()
                self.goalVelocity[robot_name] = self.goalVelocityList[robot_name].pop()
            # print "goalPosition: ", self.goalPosition[robot_name]

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        v, w = LocalPlanner.executeLocalPlanner(self.session, self.pose, self.goalPosition, self.goalVelocity)

        for idx, robot_name in enumerate(self.robotList):
            logging.debug(robot_name + '-v:' + str(v[idx]) + ' w:' + str(w[idx]))
            self.drive_handler[robot_name].setVelocity(v[idx], w[idx], self.pose[robot_name][2])

            #logging.debug("pose:" + str(pose))
            departed[robot_name] = not is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.current_regVertices[robot_name])
            # Figure out whether we've reached the destination region
            arrived[robot_name] = is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.next_regVertices[robot_name])

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

        #logging.debug("arrived:" + str(arrived))
        return (True in arrived.values()) #arrived[self.executor.hsub.getMainRobot().name]

    def createRegionPolygon(self,region,hole = None):
        """
        This function takes in the region points and make it a Polygon.
        """
        if hole == None:
            pointArray = [x for x in region.getPoints()]
        else:
            pointArray = [x for x in region.getPoints(hole_id = hole)]
        pointArray = map(self.coordmap_map2lab, pointArray)
        regionPoints = [(pt[0],pt[1]) for pt in pointArray]
        formedPolygon= Polygon.Polygon(regionPoints)
        return formedPolygon
