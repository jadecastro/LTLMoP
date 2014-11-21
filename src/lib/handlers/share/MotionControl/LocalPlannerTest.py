import pymatlab
import numpy as np
import math
import logging
from collections import OrderedDict

threshold = 10
robRadius = OrderedDict([('rob2',0.15), ('rob1',0.15)])
robMaxVel = OrderedDict([('rob2',0.5), ('rob1',0.5)])
robots = [robRadius, robMaxVel]

session = pymatlab.session_factory()

####################################
###### run initialization ##########
####################################
#function [vx, vy]=getvelocity(pose, threshold, vertices,robots,destination)
# Takes as input pose (n x d pose of all robots), threshold (how far robots
# can coordinate), vertices (vertices of all regions as a cell array), currentrobot (the
# index n_i of the current robot), and robots (an n x 1 array that contains
# the size of each robot) destination (1xn array) contains the destination
# cell for each robot

session.run('cd /home/jon/Dropbox/Repos/uav-avoidance/multiquad-sim')
session.run('settingsHadas = overwriteSettingsHadas(1);')
session.run('simLocalPlanning_initialize();')

session.run('simLocalPlanning_doStep')

session.run('[states_out, inputs_out] = readStateAndCommands(z_glob, R, vUC);')

for i in [0, 1]:
    session.run('vOut_x = inputs_out{'+str(i+1)+'}(1);')
    session.run('vOut_y = inputs_out{'+str(i+1)+'}(2);')
    session.run('vOut{'+str(i+1)+'} = inputs_out{'+str(i+1)+'}(2);')
    vx = session.getvalue('vOut_x')
    vy = session.getvalue('vOut_y')

for i, roboName in enumerate(robRadius.iteritems()):
    print i

del session