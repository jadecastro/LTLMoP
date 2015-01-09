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

import lib.handlers.handlerTemplates as handlerTemplates

class DeadlockSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):

        # robot parameters
        vMax = 0.5;
        self.prefV = 0.6*vMax
        self.stoppedStepsForDeadlock = 20  #~~10 seconds

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]

        self.drive_handler = {}
        self.pose_handler  = {}
        self.pastRobotPose = {}
        self.pastTime      = {}
        self.deadlockAgent = {}
        self.timesStoppedNotConvergedAgent  = {}
        # Get references to handlers we'll need to communicate with
        for robot_name in self.robotList: # x must be a string
            self.drive_handler[robot_name]                  = executor.hsub.getHandlerInstanceByType(handlerTemplates.DriveHandler, robot_name)
            self.drive_handler[robot_name].loco             = executor.hsub.getHandlerInstanceByType(handlerTemplates.LocomotionCommandHandler, robot_name)
            self.pose_handler[robot_name]                   = executor.hsub.getHandlerInstanceByType(handlerTemplates.PoseHandler, robot_name)
            self.pastRobotPose[robot_name]                  = []
            self.pastTime[robot_name]                       = []
            self.deadlockAgent[robot_name]                  = []
            self.timesStoppedNotConvergedAgent[robot_name]  = 0

        self.hsub= executor.hsub

        self.rfi = executor.proj.rfi

    def deadlockRobot(self, robot_id, init_value, initial=False):
        """
        Return True iff the robot is in deadlock

        robot_id (int): id of the robot, starting at 0.
        init_value (bool): The initial state of the sensor (default=False)
        """

        robot_name = self.robotList[robot_id]
        print "robot_name: "+str(robot_name)
        # get the robot's current pose
        robotPose = self.pose_handler[robot_name].getPose()

        if initial:
            self.deadlockAgent[robot_name] = init_value
            self.pastRobotPose[robot_name] = deepcopy(robotPose)
            self.pastTime[robot_name] = time.time()
            return init_value
        else:
            timeStep = time.time() - self.pastTime[robot_name] 
            vHColFree = (robotPose - self.pastRobotPose[robot_name])/timeStep  # Note: using all three components here
            # print "robotPose: "+str(robotPose)
            # print "vHColFree: "+str(vHColFree)
            # print "norm(vHColFree) : "+str(norm(vHColFree))
            # print "self.prefV/5 : "+str(self.prefV/5)

            # check if velocity small
            if (norm(vHColFree) < self.prefV/5): # could be in a deadlock?
                self.timesStoppedNotConvergedAgent[robot_name] += 1
            else:
                self.timesStoppedNotConvergedAgent[robot_name] = max(0, self.timesStoppedNotConvergedAgent[robot_name] - 3)
            
            # flag if number of consecutive steps exceeds threshold
            deadlockAgent = False
            if (self.timesStoppedNotConvergedAgent[robot_name] > self.stoppedStepsForDeadlock):
                deadlockAgent = True

            self.deadlockAgent[robot_name] = deadlockAgent
            self.pastRobotPose[robot_name] = deepcopy(robotPose)
            self.pastTime[robot_name] = time.time()
            print "deadlockAgent: "+str(deadlockAgent)
            return deadlockAgent

