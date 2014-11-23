import pymatlab
import numpy as np
import logging
from collections import OrderedDict

threshold = 10
robRadius = OrderedDict([('rob2',0.15), ('rob1',0.15)])
robMaxVel = OrderedDict([('rob2',0.5), ('rob1',0.5)])
robots = [robRadius, robMaxVel]
pathToMatlabLocalPlanner = '/home/jon/Dropbox/Repos/uav-avoidance/multiquad-sim'

def initializeLocalPlanner(regions, coordmap_map2lab):
    """
    intialize communication with MATLAB. send robotRadius, threshold and regions.
    regions: self.rfi.regions
    coordmap_map2lab: self.coordmap_map2lab
    """
    logging.info('Starting MATLAB session...')
    session = pymatlab.session_factory()
    logging.info('MATLAB Session initialized ...')

    ####################################
    ###### run initialization ##########
    ####################################
    #function [vx, vy]=getvelocity(pose, threshold, vertices,robots,destination)
    # Takes as input pose (n x d pose of all robots), threshold (how far robots
    # can coordinate), vertices (vertices of all regions as a cell array), currentrobot (the
    # index n_i of the current robot), and robots (an n x 1 array that contains
    # the size of each robot) destination (1xn array) contains the destination
    # cell for each robot

    session.run('cd '+pathToMatlabLocalPlanner)
    session.run('settingsHadas = overwriteSettingsHadas(1);')
    session.run('simLocalPlanning_initialize();')

    #-------------------------------------------------------------------
    # -----PYTHON: robotRadius, MATLAB: robots (scalar) ----------------
    #-------------------------------------------------------------------
    rRadius = []
    for roboName, propRadius in robRadius.iteritems():
        rRadius.append([propRadius])
        # rMaxVel.append([propMaxVel])
    robotRadius = np.float_(rRadius)
    # robotMaxVel = np.float_(rMaxVel)

    session.putvalue('rRob',robotRadius)
    # session.putvalue('vMax',robotMaxVel)

    logging.info('Set robotRadius completed')
    # logging.debug("  in python: " + str(robotRadius) + " , " + str(robotMaxVel))
    logging.debug("  in python: " + str(robotRadius))
    # logging.debug("  in MATLAB: " + str(session.getvalue('rRob')) + " , " str(session.getvalue('vMax')))
    logging.debug("  in MATLAB: " + str(session.getvalue('rRob')))

    #---------------------------------------------------------------------------------------
    #----- PYTHON: regionVertices, MATLAB: vertices SIZE: cell(#regions)--------------------
    #---------------------------------------------------------------------------------------

    # initialize cell array in MATLAB
    cellArrayScript = 'vertices = cell('+ str(len(regions))  +',1)'
    session.putvalue('cellArrayScript',cellArrayScript)
    session.run('eval(cellArrayScript)')

    ## send each region vertices to MATLAB
    # code for getting vertices in LTLMoP
    for regionIdx, region in enumerate(regions): #TODO: self.rfi.regions in LTLMoP and uncomment below
        logging.debug(regionIdx)
        pointArray = [y for y in region.getPoints()]
        pointArray = map(coordmap_map2lab, pointArray)
        vertices = np.mat(pointArray)
        #vertices = np.mat(region).T #TODO: remove in LTLMoP

        # add tempRegion to MATLAB vertices array
        session.putvalue('region'+str(regionIdx),np.float_(vertices))
        insertRegiontoCellScript = 'vertices{'+ str(regionIdx + 1) + '} = region'+str(regionIdx)
        session.putvalue('insertRegiontoCellScript',insertRegiontoCellScript)
        session.run('eval(insertRegiontoCellScript)')

    logging.info('Set region vertices completed')
    for regionIdx, region in enumerate(regions):
        logging.debug(session.getvalue('region'+str(regionIdx)))

    # return matlab session
    return session

def executeLocalPlanner(session, poseDic, goalPosition, goalVelocity):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """

    for i, poseLoc in enumerate(poseDic.iteritems()):
        roboName = poseLoc[0]
        session.run('pose=[];')
        session.run('zGoalNew=[];')
        session.run('vGoalNew=[];')
        
        # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)    
        session.putvalue('pose',np.float_(poseLoc[1]))
        session.run('states{'+str(i+1)+'}.position(1:2)=pose(1:2);')
        session.run('states{'+str(i+1)+'}.position(3)=1;')
        session.run('states{'+str(i+1)+'}.velocity=z_glob{'+str(i+1)+'}(4:6);') # NB: for now, use Matlab-simulated velocity.  TODO: update with LTLMoP-generated velocity
        session.run('states{'+str(i+1)+'}.orientation=pose(3);')
    
        logging.info('Set robotPose completed')
        logging.debug("  in python: " + str(np.float_(poseLoc[1])))
        logging.debug("  in MATLAB: " + str(session.getvalue('pose')))

        # Set the goal position: PYTHON: goalPosition, MATLAB: zGoal  (size 2 x n)
        session.putvalue('zGoalNew',np.float_(goalPosition[roboName]))
        session.run('zGoal{'+str(i+1)+'}(1:2)=zGoalNew(1:2);')
        session.run('zGoal{'+str(i+1)+'}(3)=1;')

        logging.info('Set goalPosition completed')
        logging.debug("  in python: " + str(np.float_(goalPosition[roboName])))
        logging.debug("  in MATLAB: " + str(session.getvalue('zGoalNew')))

        # Set the goal velocity: PYTHON: goalVelocity, MATLAB: vGoal  (size 2 x n)
        session.putvalue('vGoalNew',np.float_(goalVelocity[roboName]))
        session.run('vGoal{'+str(i+1)+'}(1:2)=vGoalNew;')

        logging.info('Set goalVelocity completed')
        logging.debug("  in python: " + str(np.float_(goalVelocity[roboName])))
        logging.debug("  in MATLAB: " + str(session.getvalue('vGoalNew')))

    # Execute one step of the local planner and collect velocity components
    session.run('[z_glob, R, zGoal] = overwriteStateAndGoal(states, zGoal);')
    session.run('simLocalPlanning_doStep')
    session.run('[states_out, inputs_out] = readStateAndCommands(z_glob, R, vUC);')

    v = {}
    w = {}
    for i, poseLoc in enumerate(poseDic.iteritems()):
        session.run('vOut = inputs_out{'+str(i+1)+'}(1);')
        session.run('wOut = inputs_out{'+str(i+1)+'}(2);')
        logging.debug('v = ' + str(session.getvalue('vOut')))
        logging.debug('w = ' + str(session.getvalue('wOut')))

        v[i] = session.getvalue('vOut')
        w[i] = session.getvalue('wOut')

    # return velocities
    return v, w

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session
