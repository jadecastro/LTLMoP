#!/usr/bin/env python
import roslib

import rospy, math, subprocess, os, sys

from geometry_msgs.msg import Twist
"""
==================================================================
rosLocomotionCommand.py - ros Locomotion Command Handler
==================================================================
"""

import lib.handlers.handlerTemplates as handlerTemplates

class YoubotLocomotionCommandHandler(handlerTemplates.LocomotionCommandHandler):
	def __init__(self, executor, shared_data, maxomega=0.1):
		"""
		The ROS Locomotion Command Handler
		maxomega (float): maximum allowed turn rate (default=0.1)
		"""

		self.maxomega = maxomega

		try:
			#open a publisher for the base controller of the robot
			self.pub = rospy.Publisher("/cmd_vel", Twist)

			# rospy.init_node("LTLMoP_control")
		except:
			print 'Problem setting up Locomotion Command Node'

	def sendCommand(self, cmd):

		#Twist is the message type and consists of x,y,z linear velocities
		#and roll, pitch, yaw orientation velocities (x,y,z)
		twist=Twist()
		twist.linear.x=cmd[0]
		twist.linear.y=cmd[1]
		twist.angular.z=min(max(cmd[2],-self.maxomega),self.maxomega)
		self.pub.publish(twist)
		try:
			#Publish the command to the robot
			self.pub.publish(twist)
		except:
			print 'Error publishing Twist Command'