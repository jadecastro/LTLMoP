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

import _pyvicon

import lib.handlers.handlerTemplates as handlerTemplates

class DeadlockSensorHandler(handlerTemplates.SensorHandler):
    def __init__(self, executor, shared_data):

        # robot parameters
        vMax = 0.08;
        self.prefV = 1*vMax
        self.stoppedStepsForDeadlock = 1  #~~10 seconds

        # get the list of robots
        self.robotList = [robot.name for robot in executor.hsub.executing_config.robots]

        self.drive_handler = {}
        self.pose_handler  = {}
        self.pastRobotPose = {}
        self.pastTime      = {}
        self.deadlockAgent = {}
        self.timesStoppedNotConvergedAgent = {}
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

        # set up a vicon stream with the helmet
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

    def deadlockRobot(self, robot_id, init_value, initial=False):
        """
        Return True iff the robot is in deadlock

        robot_id (int): id of the robot, starting at 0.
        init_value (bool): The initial state of the sensor (default=False)
        """

        robot_name = self.robotList[robot_id]
        # print "robot_name: "+str(robot_name)
        # get the robot's current pose
        robotPose = self.pose_handler[robot_name].getPose()

        # get helmet pose
        (t0, x0, y0, o0) = self.streamer.getData()
        (t0, x0, y0, o0) = [t0/100, x0/1000, y0/1000, o0]
        self.poseHelmet = [x0, y0, o0]

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
            if (norm(vHColFree) < self.prefV) and norm(self.poseHelmet[:2] - robotPose[:2]) < 0.8: # could be in a deadlock?
                logging.debug("are we in deadlock?? "+str(robot_name))
                print "are we in deadlock?? "+str(robot_name)
                self.timesStoppedNotConvergedAgent[robot_name] += 1
            else:
                self.timesStoppedNotConvergedAgent[robot_name] = max(0, self.timesStoppedNotConvergedAgent[robot_name] - 3)
            
            # flag if number of consecutive steps exceeds threshold
            deadlockAgent = False
            if (self.timesStoppedNotConvergedAgent[robot_name] > self.stoppedStepsForDeadlock):
                deadlockAgent = True
                self.timesStoppedNotConvergedAgent[robot_name]  = 0

            self.deadlockAgent[robot_name] = deadlockAgent
            self.pastRobotPose[robot_name] = deepcopy(robotPose)
            self.pastTime[robot_name] = time.time()
            logging.debug("deadlockAgent: "+str(deadlockAgent))
            return deadlockAgent

