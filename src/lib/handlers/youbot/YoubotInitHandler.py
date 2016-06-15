#!/usr/bin/env python
"""
=================================================
YoubotInitHandler.py - youBot Initialization Handler
=================================================
Initialize the proxies to access the youBot
"""
import os, rospy
import lib.handlers.handlerTemplates as handlerTemplates

class YoubotInitHandler(handlerTemplates.InitHandler):
    def __init__(self, executor,talkhost="10.0.0.135",listenhost="10.0.0.128",listenport=11311):
        """
        initialize youbot
        talkhost (string): ip address of the host computer (default="10.0.0.135")
        listenhost (string): ip address of youbot (default="10.0.0.128")
        listenport (int): port of youbot (default=11311)
        """
        #set the environmental variable ROS_MASTER_URI to enable connection between a laptop and the youBot's PC
        os.putenv('ROS_HOSTNAME','{}'.format(talkhost))
        os.putenv('ROS_IP','{}'.format(talkhost))
        os.putenv('ROS_MASTER_URI','http://{}:{}'.format(listenhost,listenport))

        rospy.init_node('LTLMoPHandlers')
        
    def getSharedData(self):
        return {'YOUBOT_INIT_HANDLER': self}

if __name__ == "__main__":
    pass