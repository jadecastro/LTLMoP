#!/usr/bin/env python
"""
=======================================================
MultiRobotLocalPlannerHandler.py 
=======================================================

"""

from numpy import *
from __is_inside import *
import time, math, sys
import logging
import __LocalPlannerHelper as LocalPlanner
from collections import OrderedDict
from scipy.linalg import norm
import Polygon, Polygon.IO
import Polygon.Utils as PolyUtils
import Polygon.Shapes as PolyShapes
import project
import pymatlab

import subprocess
import socket
import pickle

import _pyvicon

import lib.handlers.handlerTemplates as handlerTemplates
from lib.regions import Point

class MultiRobotLocalPlannerHandler(handlerTemplates.MotionControlHandler):
    def __init__(self, executor, shared_data, scalingPixelsToMeters):
        """
        Vector motion planning controller

        scalingPixelsToMeters (float): Scaling factor between RegionEditor map and Javier's map
        """
        self.numRobots              = []    # number of robots: number of agents in the specification, controlled by the local planner
        self.numDynamicObstacles    = 0     # number of dynamic obstacles: obstacles whose velocities are internally- or externally-controlled and do NOT do collision avoidance
        self.numExogenousRobots     = 0     # number of exogenous agents: robots that are controlled by another (unknown) specification, with collision avoidance
        self.robotType              = 1     # Set the robot type: quads (type 1) iCreate (type 2) and NAO (type 3)
        self.acceptanceFactor       = 4     # factor on the robot radius for achieving a goal point

        self.scenario               = 2     # 1 = garbage collection, 2 = 3D quads

        if self.scenario == 1:
            self.numberOfStepsToApplyNewGoal = 4
        elif self.scenario == 2:
            self.numberOfStepsToApplyNewGoal = 10
            self.acceptanceFactor       = 5     # factor on the robot radius for achieving a goal point

        self.scalingPixelsToMeters = scalingPixelsToMeters
        self.updateWithPose     = {}
        self.forceUpdate        = False
        self.initial            = True
        self.goal               = {}
        self.previous_next_reg  = {}
        self.previous_curr_reg  = {}
        self.pose               = {}
        self.poseExog           = {}
        self.goalPosition       = {}
        self.goalPositionExog   = {}
        self.goalVelocity       = {}
        self.goalVelocityExog   = {}
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
        self.goalPositionList = OrderedDict()
        self.goalVelocityList = OrderedDict()
        self.counter = OrderedDict()

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]
        if not self.numRobots == len(self.robotList):
            self.numRobots = len(self.robotList)
            # self.numRobots = 3
            print "WARNING: changing the number of robots to "+str(self.numRobots)+" to match the config file"
        for robot_name in self.robotList:
            self.updateWithPose[robot_name] = False
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

        # Generate bounding box for passing to the local planner
        # testSetOfRegions = self.rfi.regions    # TODO: hard-coding. decomposed region file doesn't store the boundary.
        testSetOfRegions = []
        if self.scenario == 1:
            testSetOfRegions.append([Point(0,0),Point(0,700),Point(700,0),Point(700,700)])
        if self.scenario == 2:
            testSetOfRegions.append([Point(0,0),Point(0,595),Point(490,0),Point(490,595)])
        for region in testSetOfRegions:
            regionPoints = self.getRegionVertices(region)
            print regionPoints 
            # if region.name == 'boundary':
            xv = [x for x,y in regionPoints]; yv = [y for x,y in regionPoints]
            limitsMap = [min(xv), max(xv), min(yv), max(yv)]
        # print 'Bounding box: '+str(limitsMap)
        self.limitsMap = limitsMap

        # Generate a list of obstacle vertices for the local planner
        obstacles = []
        if self.scenario == 1:
            obstacles.append([Point(540,490),Point(540,700),Point(700,490),Point(700,700)])
            obstacles.append([Point(574,210),Point(574,280),Point(700,210),Point(700,280)])
            obstacles.append([Point(385,210),Point(385,280),Point(465,210),Point(465,280)])
            obstacles.append([Point(105,210),Point(105,490),Point(385,210),Point(385,490)])
        if self.scenario == 2:
            obstacles.append([Point(280,105),Point(280,245),Point(385,105),Point(385,245)])
        # obstacles = [r for r in self.rfi.regions if r.name.startswith('o')]   # TODO: hard-coding. decomposed region file doesn't store obstacles.
        obstaclePoints = []
        for region in obstacles:
            obsPoints = self.getRegionVertices(region)
            xv = [x for x,y in obsPoints]; yv = [y for x,y in obsPoints]
            obstaclePoints.append(hstack([xv,yv]))
        # print "Obstacle points: "+str(obstaclePoints)

        # Generate a set of transition faces for the local planner
        regionTransitionFaces = []
        for regionIdx, region in enumerate(self.rfi.regions):
            if not region in obstacles:
                # vertices = self.getRegionVertices(region)
                print region.name
                for nextRegionIdx, nextRegion in enumerate(self.rfi.transitions[regionIdx]):
                    if self.rfi.transitions[regionIdx][nextRegionIdx]:
                        tmp = [float(1)/self.scalingPixelsToMeters*x for x in self.rfi.transitions[regionIdx][nextRegionIdx][0]]
                        tmp1 = (tmp[0].x,tmp[0].y); tmp2 = (tmp[1].x,tmp[1].y);
                        regionTransitionFaces.append([regionIdx+1,nextRegionIdx+1,tmp1[0],tmp2[0],tmp1[1],tmp2[1]])
        # print "Transition faces: "+str(regionTransitionFaces)

        # setup matlab communication
        self.session = executor.proj.session
        # self.session = pymatlab.session_factory()

        LocalPlanner.initializeLocalPlanner(self.session, self.rfi.regions, regionTransitionFaces, obstaclePoints, self.scalingPixelsToMeters, limitsMap, \
            self.numRobots, self.numDynamicObstacles, self.numExogenousRobots, self.robotType, self.scenario)

        # set up a client session if we have other agents to avoid
        if self.numExogenousRobots > 1:
            address = ('localhost', 9999)  # let the kernel give us a port
            print "Starting the client..."

            # Create a socket (SOCK_STREAM means a TCP socket)
            self.c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.c.connect(address)

        # set up vicon feed of helmet data
        elif self.numDynamicObstacles > 0:
            print "Subscribing to the youbot..."
            self.host = "10.0.0.102"
            self.port = 800
            self.x = "KUKAyouBot2:main body <t-X>"
            self.y = "KUKAyouBot2:main body <t-Y>"
            self.theta = "KUKAyouBot2:main body <a-Z>"
            # self.x = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01 <t-X>"
            # self.y = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01 <t-Y>"
            # self.theta = "GPSReceiverHelmet-goodaxes:GPSReceiverHelmet01 <a-Z>"

            self.streamer = _pyvicon.ViconStreamer()
            self.streamer.connect(self.host,self.port)

            self.streamer.selectStreams(["Time", self.x, self.y, self.theta])

            self.streamer.startStreams()

            self.poseExog[0] = array([0.,0.,0.])
            print "Subscribed..."

    def _stop(self):
        """
        Properly terminates all threads/computations in the handler. Leave no trace behind.
        """

        session.run('simLocalPlanning_saveData();')
        logging.debug("Closing the connection...") 
        self.c.close()


    def _duplicateProject(self, proj):
        """ Creates a copy of a proj, and creates an accompanying spec file with an 
            auto-incremented counter in the name.  (Not overwriting is mostly for debugging.)"""

        # reload from file instead of deepcopy because hsub stuff can include uncopyable thread locks, etc
        new_proj = project.Project()
        new_proj.setSilent(True)
        new_proj.loadProject(self.executor.proj.getFilenamePrefix() + ".spec")

        # copy hsub references manually
        # new_proj.hsub = proj.executor.hsub
        # new_proj.hsub.proj = new_proj # oh my god, guys
        new_proj.h_instance = proj.h_instance
        
        # new_proj.sensor_handler = proj.sensor_handler
        # new_proj.actuator_handler = proj.actuator_handler

        return new_proj

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
                    self.drive_handler[robot_name].setVelocity(0, 0)  # So let's stop
                    time.sleep(1)
                    #return False not leaving yet until all robots are checked

                # if self.system_print == True:
                print "Next Region is " +str(robot_name)+str(self.rfi.regions[next_reg].name)
                print "Current Region is " +str(robot_name)+str(self.rfi.regions[current_reg].name)
                logging.debug("Next Region is " + str(self.rfi.regions[next_reg].name))
                logging.debug("Current Region is " + str(self.rfi.regions[current_reg].name))

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
                        if self.robotType == 3:
                            goal1 = goalArrayNew[:,i]-face_normalNew[:,i]*1.5*self.radius    ##original 2*self.radius
                            goal2 = goalArrayNew[:,i]+face_normalNew[:,i]*1.5*self.radius    ##original 2*self.radius
                        else:
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
                    print "CHANGING REGIONS. contents of the position goal list: "+str(robot_name)+str(self.goalPositionList[robot_name])
                    self.updateWithPose[robot_name] = False
                    
                else:
                    # self.goalPositionList[robot_name] = []
                    # self.goalVelocityList[robot_name] = []
                    print "STAYING IN REGION. contents of the position goal list: "+str(robot_name)+str(self.goalPositionList[robot_name])+str(len(self.goalPositionList[robot_name]))
                    if len(self.goalPositionList[robot_name]) > 0:
                        self.goalPositionList[robot_name].pop()
                        self.goalVelocityList[robot_name].pop()  
                    # else:
                    self.goalPositionList[robot_name].append(self.pose[robot_name][:2]+[0.1,-0.1])
                    self.goalVelocityList[robot_name].append([0, 0])  # temporarily setting this to zero
                    # goal = []
                    goal = self.goal[robot_name]  # TODO: fix the case of a self-loop in the initial region
                    doUpdate[robot_name] = True
                    self.updateWithPose[robot_name] = True
                    

                self.goal[robot_name] = goal

                # initialize the goal to the current pose, if the initial goal list is empty
                if len(self.goalPositionList[robot_name]) == 0 and self.initial:
                    print "Initial condition: STAYING IN REGION "+str(robot_name)
                    self.goalPositionList[robot_name].append(self.pose[robot_name][:2]+[0.1,-0.1])
                    # self.goalPositionList[robot_name].append(self.pose[robot_name][:2]+[40,-40])
                    self.goalVelocityList[robot_name].append([0, 0])

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

            # if len(self.goalPosition[robot_name]) > 0:
            #     print "norm: ", norm(mat(self.pose[robot_name][:2]) - self.goalPosition[robot_name])
            #     print "diff: ", mat(self.pose[robot_name][:2]).T - self.goalPosition[robot_name]
            # print "pose: ", self.pose[robot_name][:2]
            # print self.goalPosition[robot_name]
            # print all(self.goalPosition[robot_name])
            # print "current goal: "+str(self.goalPosition[robot_name])
            # print "list of goals: "+str(self.goal[robot_name])
            # print "next goal position: "+str(self.goalPositionList[robot_name])
            # print "distance to goal:"+str(robot_name)
            # print norm(mat(self.pose[robot_name][:2]).T - self.goalPosition[robot_name]) 
            # print self.acceptanceFactor*self.radius
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
                    print "starting counter "+str(robot_name)
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
                print "counter increment "+str(robot_name)
                self.counter[robot_name]  += 1
            if not self.initial and self.counter[robot_name]  > self.numberOfStepsToApplyNewGoal: #and current_regIndices[robot_name] == next_regIndices[robot_name]:
                print "UPDATING GOAL. contents of the position goal list: "+str(robot_name)+str(self.goalPositionList[robot_name])
                if self.updateWithPose[robot_name]:
                    print " using current pose"+str(self.pose[robot_name][:2]+[0.01,-0.01])
                    self.goalPosition[robot_name] = self.pose[robot_name][:2]+[0.01,-0.01] 
                    self.goalVelocity[robot_name] = self.goalVelocityList[robot_name].pop(0)
                    self.goalPositionList[robot_name].pop(0)
                else:
                    self.goalPosition[robot_name] = self.goalPositionList[robot_name].pop(0)
                    self.goalVelocity[robot_name] = self.goalVelocityList[robot_name].pop(0)   

                self.updateWithPose[robot_name] = False                
                   
                self.counter[robot_name] = 0
                # self.goalPosition[robot_name] = self.pose[robot_name][:2]
                doUpdate[robot_name] = True
                print "new zgoal: "+str(robot_name)+str(self.goalPosition[robot_name] )

            # print "goalPosition: ", self.goalPosition[robot_name]

        # receive the goal, pose, and velocity for the dynamic obstacles.
        if self.numExogenousRobots > 1:
            try:
                # Receive data from the server
                response = pickle.loads(self.c.recv(1024))
                # print "Got message {}".format(response)

            except socket.error, msg:
                print "Socket error! %s" % msg

            # parse the incoming pose info
            for i in range(self.numExogenousRobots):
                self.poseExog[i] = array(response[i][0]) 
                self.goalPositionExog[i] = response[i][1]
                self.goalVelocityExog[i] = [0.,0.]

        elif self.numDynamicObstacles > 0:
            (t0, x0, y0, o0) = self.streamer.getData()
            (t0, x0, y0, o0) = [t0/100, x0/1000, y0/1000, o0]
            if x0 == 0. and y0 == 0:
                print "WARNING: No helmet vicon data. Re-using old data."
            elif (x0 < self.limitsMap[0] or x0 > self.limitsMap[1] or y0 < self.limitsMap[2] or y0 > self.limitsMap[3]):
                print "WARNING: Helmet outside of map bounds. Re-using old data."
            else:
                self.poseExog[0] = array([x0, y0, o0])
            self.goalPositionExog[0] = [0.,0.]
            self.goalVelocityExog[0] = [0.,0.]

        # Run algorithm to find a velocity vector (global frame) to take the robot to the next region
        v, w, vd, wd, deadAgent = LocalPlanner.executeLocalPlanner(self.session, self.pose, self.goalPosition, self.goalVelocity, self.poseExog, self.goalPositionExog, self.goalVelocityExog, \
            doUpdate, self.rfi.regions, current_regIndices, next_regIndices, self.coordmap_lab2map, self.scalingPixelsToMeters, self.numDynamicObstacles, self.scenario)

        # save the data
        if (time.time() - self.timer) > 10:
            self.timer = time.time()
            self.session.run('simLocalPlanning_saveData();')

        # send the v and w for the dynamic obstacles
        if self.numExogenousRobots > 1:
            try:
                # Send data to the server
                message = [vd, wd]
                self.c.sendall(pickle.dumps(message))

            except socket.error, msg:
                print "Socket error! %s" % msg

        # If in deadlock, hot-swap a new automaton
        # logging.debug("deadAgent: "+str(deadAgent))
        # autpath = "/home/jon/Dropbox/Repos/LTLMoP/src/examples/local_planner_hotswap/aut/"
        # if any(deadAgent):
        #     print "Hot-swapping a new aut..."
        #     if deadAgent[0] and not deadAgent[1]:
        #         j = 0
        #     elif not deadAgent[0] and deadAgent[1]:
        #         j = 1
        #     elif deadAgent[0] and deadAgent[1]:
        #         j = 1
        #     robot_name = self.robotList[j]
        #     current_reg = current_regIndices[robot_name]
        #     next_reg = next_regIndices[robot_name]
        #     currentName = self.rfi.regions[current_regIndices[robot_name]].name
        #     nextName = self.rfi.regions[next_regIndices[robot_name]].name
            
        #     if "T" in currentName and "L" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_T_L.aut"
        #     elif "T" in currentName and "R" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_T_R.aut"
        #     elif "B" in currentName and "L" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_B_L.aut"
        #     elif "B" in currentName and "D" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_B_D.aut"
        #     elif "L" in currentName and "T" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_L_T.aut"
        #     elif "L" in currentName and "B" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_L_B.aut"
        #     elif "R" in currentName and "T" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_R_T.aut"
        #     elif "R" in currentName and "D" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_R_D.aut"
        #     elif "D" in currentName and "R" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_D_R.aut"
        #     elif "D" in currentName and "B" in nextName:
        #         aut_file =  autpath+"local_planner_hotswap_rob"+str(j+1)+"_D_B.aut"

        #     self.executor.pause()

        #     # Copy the current project
        #     print self.executor.proj.__dict__
        #     new_proj = self._duplicateProject(self.executor.proj)

        #     # Swap in the new automaton
        #     self.executor.proj = new_proj
        #     self.executor.initialize("/home/jon/Dropbox/Repos/LTLMoP/src/examples/local_planner_deadlocks/local_planner_deadlocks.spec", aut_file, firstRun=False)
        #     # aut_file = self.proj.getFilenamePrefix() + ".aut"
        #     # self.initialize(spec_file, aut_file, firstRun=False)

        #     self.executor.resume()

        for idx, robot_name in enumerate(self.robotList):
            if (self.robotType == 1 or self.robotType == 3) and current_regIndices[robot_name] == next_regIndices[robot_name]:
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
