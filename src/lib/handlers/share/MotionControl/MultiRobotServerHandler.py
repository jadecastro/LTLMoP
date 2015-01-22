#!/usr/bin/env python
"""
=======================================================
MultiRobotServerHandler.py 
=======================================================

"""

from numpy import *
from __is_inside import *
import time, math
import logging
from collections import OrderedDict
from scipy.linalg import norm
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes

import socket
import subprocess
import select
import os
import sys
import globalConfig
import pickle

import lib.handlers.handlerTemplates as handlerTemplates
from lib.regions import Point


class MultiRobotServerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data, scalingPixelsToMeters):
        """
        Vector motion planning controller

        scalingPixelsToMeters (float): Scaling factor between RegionEditor map and Javier's map
        """

        self.numExogenousRobots     = 2    # number of exogenous agents: robots that are controlled by another (unknown) specification, with collision avoidance
        self.acceptanceFactor       = 4     # factor on the robot radius for achieving a goal point

        self.scalingPixelsToMeters = scalingPixelsToMeters
        self.forceUpdate        = False
        self.initial            = True
        self.goal               = {}
        self.previous_next_reg  = {}
        self.pose               = {}
        self.goalPosition       = {}
        self.goalVelocity       = {}
        self.current_regVertices= {}
        self.next_regVertices   = {}
        self.system_print       = False       # for debugging. print on GUI ( a bunch of stuffs)
        self.Velocity           = None
        self.currentRegionPoly  = None
        self.nextRegionPoly     = None
        self.radius             = self.scalingPixelsToMeters*0.15
        self.trans_matrix       = mat([[0,1],[-1,0]])   # transformation matrix for find the normal to the vector
        # self.response           = [1.,1.]
        # self.message            = {}
        self.v                  = self.numExogenousRobots*[0.]
        self.w                  = self.numExogenousRobots*[0.]
        self.timer              = time.time()

        self.pose = OrderedDict()
        self.goalPositionList = OrderedDict()
        self.goalVelocityList = OrderedDict()
        self.counter = OrderedDict()

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        for robot_name in self.robotList:
            self.goalPosition[robot_name] = []
            self.goalVelocity[robot_name] = []
            self.goalPositionList[robot_name] = []
            self.goalVelocityList[robot_name] = []
            self.counter[robot_name] = 0

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

        # set the initial goal as the initial pose, in case there are no transitions to take immediately
        for robot_name in self.robotList: 
        #     self.pose.update([(robot_name,self.pose_handler[robot_name].getPose())])
            self.goal[robot_name] = []

        # start the server
        address = ('localhost', 9999)  # let the kernel give us a port
        print "Starting the server..."

        # Connect to the server
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(address)

        print "Server started. Waiting for client..."
        self.s.listen(5)                 # Now wait for client connection.
        self.client, self.addr = self.s.accept()

    def _stop(self):
        """
        Properly terminates all threads/computations in the handler. Leave no trace behind.
        """
        logging.debug("Closing the connection...") 
        self.s.close()


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
        doUpdate            = {}

        logging.debug("current_regIndices:" + str(current_regIndices))
        logging.debug("next_regIndices: " + str(next_regIndices))

        if not self.previous_next_reg:
            for robot_name, current_reg in current_regIndices.iteritems():
                self.previous_next_reg[robot_name] = []

        self.message = []
        for robot_name, current_reg in current_regIndices.iteritems():
            next_reg = next_regIndices[robot_name]

            doUpdate[robot_name] = False            
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

                print "self.rfi.regions[current_reg].name: ",self.rfi.regions[current_reg].name
                self.currentRegionPoly = self.map[self.rfi.regions[current_reg].name]
                regionPolyOld = Polygon.Polygon(self.currentRegionPoly)

                if not current_reg == next_reg:
                    if last:
                        transFace = None
                    else:
                        # Determine the mid points on the faces connecting to the next region
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
                    doUpdate[robot_name] = True

                    i = 0
                    goal = 2*[[]]
                    while i < goalArrayNew.shape[1]:
                        goal1 = goalArrayNew[:,i]-face_normalNew[:,i]*3*self.radius    ##original 2*self.radius
                        goal2 = goalArrayNew[:,i]+face_normalNew[:,i]*3*self.radius    ##original 2*self.radius
                        if regionPolyOld.isInside(float(1)/self.scalingPixelsToMeters*goal1[0], float(1)/self.scalingPixelsToMeters*goal1[1]):
                            goal[0] = goal1
                            goal[1] = goal2
                        else:
                            goal[0] = goal2
                            goal[1] = goal1
                        i += 1

                    self.goalPositionList[robot_name] = []
                    self.goalVelocityList[robot_name] = []
                    for i in range(len(goal)):
                        self.goalPositionList[robot_name].append(goal[i])  #for now, assume there is only one face.
                        self.goalVelocityList[robot_name].append([0, 0])  # temporarily setting this to zero

                else:
                    goal = self.goal[robot_name]  # TODO: fix the case of a self-loop in the initial region
                
                self.goal[robot_name] = goal

                # initialize the goal to the current pose, if the initial goal list is empty
                if len(self.goalPositionList[robot_name]) == 0 and self.initial:
                    self.goalPositionList[robot_name].append(self.pose[robot_name][:2]+[0.1,-0.1])
                    self.goalVelocityList[robot_name].append([0, 0])

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
            # print "goalPosition: "+str(self.goalPosition[robot_name])
            # print "list of goals: "+str(self.goal[robot_name])
            # print "goalPositionList: "+str(self.goalPositionList[robot_name])
            if len(self.goalPosition[robot_name]) > 0:
                if norm(mat(self.pose[robot_name][:2]).T - self.goalPosition[robot_name]) > self.acceptanceFactor*self.radius or len(self.goalPositionList[robot_name]) == 0:
                    pass
                else:
                    doUpdate[robot_name] = True
            else: 
                doUpdate[robot_name] = True
            if doUpdate[robot_name] or self.forceUpdate:
                self.forceUpdate = False
                print "updating goal for robot "+str(robot_name)+"!!!"
                # if not self.initial and current_regIndices[robot_name] == next_regIndices[robot_name]:
                if not self.initial:
                    print "counter increment"
                    self.counter[robot_name] += 1
                    doUpdate[robot_name] = False
                else:
                    self.goalPosition[robot_name] = self.goalPositionList[robot_name].pop(0)
                    self.goalVelocity[robot_name] = self.goalVelocityList[robot_name].pop(0)
               
                    # print "current goal: "+str(self.goalPosition[robot_name])
                    # print "list of goals: "+str(self.goal[robot_name])
                    # print "next goal position: "+str(self.goalPositionList[robot_name])
               
                # if not self.initial and current_regIndices[robot_name] == next_regIndices[robot_name]:
                #     print "counter increment"
                #     self.counter += 1
            if not self.initial and self.counter[robot_name]  > 0: #and current_regIndices[robot_name] == next_regIndices[robot_name]:
                print "counter increment"
                self.counter[robot_name]  += 1
            if not self.initial and self.counter[robot_name]  > 4: #and current_regIndices[robot_name] == next_regIndices[robot_name]:
                self.goalPosition[robot_name] = self.goalPositionList[robot_name].pop(0)
                self.goalVelocity[robot_name] = self.goalVelocityList[robot_name].pop(0)

                self.counter[robot_name] = 0
                # self.goalPosition[robot_name] = self.pose[robot_name][:2]
                doUpdate[robot_name] = True
                print "new zgoal: "+str(self.goalPosition[robot_name] )

            # prepare the message to be sent to the client
            self.message.append([list(self.pose[robot_name]), self.goalPosition[robot_name].reshape(-1,).tolist()[0]])

        try:
            # Send the data: goal position, current pose, and current velocities
            # print 'Sending : "%s"' % self.message
            len_sent = self.client.send(pickle.dumps(self.message))

            # Receive a response: v and w arrays
            if not self.numExogenousRobots == 1:
                self.response = pickle.loads(self.client.recv(1024))
                # print 'Received: "%s"' % self.response

                v = self.response[0]
                w = self.response[1]
            else:
                v = [0.]
                w = [0.]
            self.v = v; self.w = w

            # set the velocities
            for idx, robot_name in enumerate(self.robotList):
                if current_regIndices[robot_name] == next_regIndices[robot_name]:
                    v[idx] = 0.
                    w[idx] = 0.
                
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

            self.initial = False
            logging.debug("arrived:" + str(arrived))
            return (True in arrived.values()) #arrived[self.executor.hsub.getMainRobot().name]

        except socket.error, msg:
            print "Socket error! %s" % msg


        # for idx, robot_name in enumerate(self.robotList):
            # logging.debug(robot_name + '-v:' + str(self.v[idx]) + ' w:' + str(self.w[idx]))
            # self.drive_handler[robot_name].setVelocity(self.v[idx], self.w[idx], self.pose[robot_name][2])
            #logging.debug("pose:" + str(pose))
            # departed[robot_name] = not is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.current_regVertices[robot_name])
            # # Figure out whether we've reached the destination region
            # arrived[robot_name] = is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], self.next_regVertices[robot_name])

            # if departed[robot_name] and (not arrived[robot_name]) and (time.time()-self.last_warning) > 0.5:
            #     #print "WARNING: Left current region but not in expected destination region"
            #     # Figure out what region we think we stumbled into
            #     for r in self.rfi.regions:
            #         pointArray = [self.coordmap_map2lab(x) for x in r.getPoints()]
            #         vertices = mat(pointArray).T
            #         if is_inside([self.pose[robot_name][0], self.pose[robot_name][1]], vertices):
            #             logging.info("I think I'm in " + r.name)
            #             break
            #     self.last_warning = time.time()

        # logging.debug("arrived:" + str(arrived))
        # return (True in arrived.values()) #arrived[self.executor.hsub.getMainRobot().name]

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

