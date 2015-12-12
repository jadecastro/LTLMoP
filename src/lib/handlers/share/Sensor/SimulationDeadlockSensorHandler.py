#!/usr/bin/env python
"""
=====================================
DeadlockSensorHandler.py - Deadlock Sensor Handler
=====================================

Detects the case where a robot is not moving.
"""

import threading, subprocess, os, time, socket
from struct import pack,unpack
from numpy import *
from scipy.linalg import norm
from copy import deepcopy
import logging

import subprocess
import socket
import pickle

import lib.handlers.handlerTemplates as handlerTemplates

class SimulationDeadlockSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):

        self.getFromOtherLTLMoPinstance = False

        # robot parameters
        vMax = 70*0.5;
        self.prefV = 0.03*vMax
        if ~self.getFromOtherLTLMoPinstance:
            self.stoppedStepsForDeadlock = 1  #immediately
        else:
            self.stoppedStepsForDeadlock = 4  #~~10 seconds
        self.deadDist = 70*1.2

        self.numExogenousRobots = 0


        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]

        self.drive_handler = {}
        self.pose_handler  = {}
        self.pastRobotPose = {}
        self.pastRobotPose1= {}
        self.pastRobotPose2= {}
        self.pastTime      = {}
        self.deadlockAgent = {}
        self.deadlockTimer = {}
        self.timesStoppedNotConvergedAgent  = {}
        self.pastTimePair  = {}
        self.deadlockAgentPair = {}
        self.timesStoppedNotConvergedAgentPair  = {}
        # Get references to handlers we'll need to communicate with
        for robot_name in self.robotList: # x must be a string
            self.drive_handler[robot_name]                  = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler, robot_name)
            self.drive_handler[robot_name].loco             = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler, robot_name)
            self.pose_handler[robot_name]                   = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler, robot_name)
            self.pastRobotPose[robot_name]                  = []
            self.pastRobotPose1[robot_name]                 = []
            self.pastRobotPose2[robot_name]                 = []
            self.pastTime[robot_name]                       = []
            self.deadlockAgent[robot_name]                  = False
            self.deadlockTimer[robot_name]                  = 0
            self.timesStoppedNotConvergedAgent[robot_name]  = 0
            self.pastTimePair[robot_name]                   = []
            self.deadlockAgentPair[robot_name]              = []
            self.timesStoppedNotConvergedAgentPair[robot_name] = 0

        self.hsub= executor.hsub

        self.rfi = executor.proj.rfi

        if self.getFromOtherLTLMoPinstance:
            # set up a client session if we have other agents to avoid
            if self.numExogenousRobots > 1:
                address = ('localhost', 9999)  # let the kernel give us a port
                print "Starting the client..."

                # Create a socket (SOCK_STREAM means a TCP socket)
                self.c = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.c.connect(address)
        else:
            self.session = executor.proj.session

    def deadlockRobot(self, robot_id, init_value, initial=False):
        """
        Return True iff the robot is in deadlock

        robot_id (int): id of the robot, starting at 0.
        init_value (bool): The initial state of the sensor (default=False)
        """

        robot_name = self.robotList[robot_id]
        #print "deadlockRobot: robot_name: "+str(robot_name)
        # get the robot's current pose
        robotPose = self.pose_handler[robot_name].getPose()

        # get helmet pose
        # receive the goal, pose, and velocity for the dynamic obstacles.
        if self.numExogenousRobots > 1:
            try:
                # Receive data from the server
                print "Getting message"
                response = pickle.loads(self.c.recv(1024))
                print "Got message {}".format(response)

            except socket.error, msg:
                print "Socket error! %s" % msg

            # parse the incoming pose info
            for i in range(self.numExogenousRobots):
                self.poseExog[i] = array(response[i][0])

        if initial:
            self.deadlockAgent[robot_name] = init_value
            self.pastRobotPose[robot_name] = deepcopy(robotPose)
            self.pastTime[robot_name] = time.time()
            self.deadlockTimer[robot_name] = 0
            return init_value
        else:
            timeStep = time.time() - self.pastTime[robot_name] 
            vHColFree = (robotPose - self.pastRobotPose[robot_name])/timeStep  # Note: using all three components here
            # print "robotPose: "+str(robotPose)
            # print "vHColFree: "+str(vHColFree)
            # print "norm(vHColFree) : "+str(norm(vHColFree))
            # print "self.prefV/5 : "+str(self.prefV/5)

            if self.getFromOtherLTLMoPinstance:
                # check if velocity small AND any exogenous agents are within a certain radius
                isWithinRadius = False
                for i in range(self.numExogenousRobots):
                    isWithinRadius |= norm(self.poseExog[i][:2] - robotPose[:2]) < self.deadDist

                if (norm(vHColFree) < self.prefV) and isWithinRadius: # could be in a deadlock?
                    logging.debug("possible deadlock detected for "+str(robot_name))
                    print "possible deadlock detected for "+str(robot_name)
                    self.timesStoppedNotConvergedAgent[robot_name] += 1
                else:
                    self.timesStoppedNotConvergedAgent[robot_name] = max(0, self.timesStoppedNotConvergedAgent[robot_name] - 3)

            else:
                # rely on deadlock flag computed in Matlab
                if self.session.getvalue('deadlockAgent'+str(robot_id+1)):
                    logging.debug("possible deadlock detected for "+str(robot_name))
                    print "possible deadlock detected for "+str(robot_name)
                    self.timesStoppedNotConvergedAgent[robot_name] += 1
                else:
                    self.timesStoppedNotConvergedAgent[robot_name] = max(0, self.timesStoppedNotConvergedAgent[robot_name] - 3)
            
            # flag if number of consecutive steps exceeds threshold
            deadlockAgent = False
            logging.debug("self.deadlockAgent: "+str(self.deadlockAgent[robot_name]))
            if (self.timesStoppedNotConvergedAgent[robot_name] >= self.stoppedStepsForDeadlock) and not self.deadlockAgent[robot_name] and (self.deadlockTimer[robot_name] > 2):
                # deadlock is set if the number of consecutive signals exceeds a threshold
                # and if deadlock hasn't been sensed in the prior step (to keep the simulation running in the event of unresolvable deadlocks)
                deadlockAgent = True
                self.timesStoppedNotConvergedAgent[robot_name] = 0
                self.deadlockTimer[robot_name] = 0
            else:
                deadlockAgent = False

            self.deadlockAgent[robot_name] = deadlockAgent
            self.pastRobotPose[robot_name] = deepcopy(robotPose)
            self.pastTime[robot_name] = time.time()
            self.deadlockTimer[robot_name] += 1
            return deadlockAgent

    def deadlockRobotPairwise(self, robot_id_1, robot_id_2, init_value, initial=False):
        """
        Return True iff the robot is in deadlock

        robot_id_1 (int): id of a robot, starting at 0.
        robot_id_2 (int): id of another robot, starting at 0.
        init_value (bool): The initial state of the sensor (default=False)
        """

        robot_name_1 = self.robotList[robot_id_1]
        robot_name_2 = self.robotList[robot_id_2]
        # print "robot_name: "+str(robot_name)
        # get the robot's current pose
        robotPose1 = self.pose_handler[robot_name_1].getPose()
        robotPose2 = self.pose_handler[robot_name_2].getPose()

        if initial:
            self.deadlockAgentPair[robot_name_1] = init_value
            self.pastRobotPose1[robot_name_1] = deepcopy(robotPose1)
            self.pastRobotPose2[robot_name_2] = deepcopy(robotPose2)
            self.pastTimePair[robot_name_1] = time.time()
            return init_value
        else:
            timeStep = time.time() - self.pastTimePair[robot_name_1]
            vHColFree1 = (robotPose1 - self.pastRobotPose1[robot_name_1])/timeStep  # Note: using all three components here
            vHColFree2 = (robotPose2 - self.pastRobotPose2[robot_name_2])/timeStep  # Note: using all three components here
            # print "robotPose: "+str(robotPose)
            # print "vHColFree: "+str(vHColFree)
            # print "norm(vHColFree) : "+str(norm(vHColFree))
            # print "self.prefV/5 : "+str(self.prefV/5)

            # check if velocity small
            # print "vel1, vel2, dist:"
            # print norm(vHColFree1)
            # print norm(vHColFree2)
            # print norm(robotPose1[:2] - robotPose2[:2])
            if (norm(vHColFree1) < self.prefV) and (norm(vHColFree2) < self.prefV) and norm(robotPose1[:2] - robotPose2[:2]) < self.deadDist: # could be in a deadlock?
                logging.debug("are we in PAIRWISE deadlock?? "+str(robot_name_1)+str(robot_name_2))
                print "are we in PAIRWISE deadlock?? "+str(robot_name_1)+str(robot_name_2)
                self.timesStoppedNotConvergedAgentPair[robot_name_1] += 1
            else:
                self.timesStoppedNotConvergedAgentPair[robot_name_1] = max(0, self.timesStoppedNotConvergedAgentPair[robot_name_1] - 3)
            
            # flag if number of consecutive steps exceeds threshold
            deadlockAgent = False
            if (self.timesStoppedNotConvergedAgentPair[robot_name_1] > self.stoppedStepsForDeadlock):
                deadlockAgent = True
                self.timesStoppedNotConvergedAgentPair[robot_name_1] = 0

            self.deadlockAgentPair[robot_name_1] = deadlockAgent
            self.pastRobotPose1[robot_name_1] = deepcopy(robotPose1)
            self.pastRobotPose2[robot_name_2] = deepcopy(robotPose2)
            self.pastTimePair[robot_name_1] = time.time()
            logging.debug("PAIRWISE deadlockAgent: "+str(deadlockAgent))
            return deadlockAgent

