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
session.run('settingsHadas = 1;')
session.run('simLocalPlanning_initialize();')


for i in [0, 1]:
    session.run('pose=[];')
    session.run('zGoalNew=[];')
    session.run('vGoalNew=[];')
    
    # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)    
    # session.putvalue('pose',np.float_([ 333.375,  290.75,     0.   ]))
    # session.run('z_glob=[z_glob z_globNew];')
    # session.run('states{'+str(i+1)+'}.position(1:2)=pose(1:2);')
    session.run('states{'+str(i+1)+'}.position=z_glob{'+str(i+1)+'}(1:3);')
    # session.run('states{'+str(i+1)+'}.position(3)=0;')
    session.run('states{'+str(i+1)+'}.velocity=z_glob{'+str(i+1)+'}(4:6);') # NB: for now, use Matlab-simulated velocity.  TODO: update with LTLMoP-generated velocity
    # session.run('states{'+str(i+1)+'}.orientation=pose(3);')
    session.run('states{'+str(i+1)+'}.orientation=rotationMatrix_2_orientation(R{'+str(i+1)+'});')

    session.run('positionOut=states{'+str(i+1)+'}.position;')
    session.run('velocityOut=states{'+str(i+1)+'}.velocity;')
    session.run('orientationOut=states{'+str(i+1)+'}.orientation;')
    print('Set robotPose completed')
    print("  in MATLAB: " + str(session.getvalue('positionOut')))
    print("  in MATLAB: " + str(session.getvalue('velocityOut')))
    print("  in MATLAB: " + str(session.getvalue('orientationOut')))

    # Set the goal position: PYTHON: goalPosition, MATLAB: zGoal  (size 2 x n)
    # session.putvalue('zGoalNew',np.float_([ 173.,  269.]))
    session.run('zGoalNew=zGoal{'+str(i+1)+'};')

    print('Set goalPosition completed')
    print("  in MATLAB: " + str(session.getvalue('zGoalNew')))

    # Set the goal velocity: PYTHON: goalVelocity, MATLAB: vGoal  (size 2 x n)
    # session.putvalue('vGoalNew',np.float_([ 0.,  0.]))
    # session.run('vGoal{'+str(i+1)+'}(1:2)=vGoalNew;')

    # print('Set goalVelocity completed')
    # print("  in MATLAB: " + str(session.getvalue('vGoal{'+str(i+1)+'}')))

session.run('[z_glob, R, zGoal] = overwriteStateAndGoal(states, zGoal);')
session.run('simLocalPlanning_doStep')
session.run('[states_out, inputs_out] = readStateAndCommands(z_glob, R, vUC);')

for i in [0, 1]:
    session.run('vOut_x = inputs_out{'+str(i+1)+'}(1);')
    session.run('vOut_y = inputs_out{'+str(i+1)+'}(2);')
    session.run('vOut{'+str(i+1)+'} = inputs_out{'+str(i+1)+'}(2);')
    vx = session.getvalue('vOut_x')
    vy = session.getvalue('vOut_y')
    print vx
    print vy

for i, roboName in enumerate(robRadius.iteritems()):
    print i

del session