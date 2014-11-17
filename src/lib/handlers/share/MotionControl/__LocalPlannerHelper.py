import pymatlab
import numpy as np
import logging
from collections import OrderedDict

threshold = 10
robRadiusMaxVel = OrderedDict([('rob2',0.15,0.5), ('rob1',0.15,0.5)])
#robRadius = OrderedDict([('rob1',0.5), ('rob2',0.5),('rob3',1)])
robots = robRadiusMaxVel

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

    session.run('eval(initializeLocalPlanner)')

    #-------------------------------------------------------------------
    # -----PYTHON: robotRadius, MATLAB: robots (scalar) ----------------
    #-------------------------------------------------------------------
    rRadius = []
    for roboName, propRadius, propMaxVel in robots.iteritems():
        rRadius.append([propRadius])
        rMaxVel.append([propMaxVel])
    robotRadius = np.float_(rRadius)
    robotMaxVel = np.float_(rMaxVel)

    session.putvalue('rRob',robotRadius)
    session.putvalue('vMax',robotMaxVel)

    logging.info('Set robotRadius completed')
    logging.debug("in python: " + str(robotRadius) + " , " + str(robotMaxVel))
    logging.debug("in MATLAB: " + str(session.getvalue('rRob')) + " , " str(session.getvalue('vMax')))

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

    session.run('zAux=[];')
    session.run('zGoal=[];')
    session.run('vGoal=[];')
    for roboName, poseLoc in poseDic.iteritems():
        
        # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)      
        session.putvalue('zAuxNew',np.float_(poseLoc))
        session.run('zAux=[zAux zAuxNew];')
        logging.info('Set robotPose completed')
        logging.debug("in python: " + str(np.float_(poseLoc)))
        logging.debug("in MATLAB: " + str(session.getvalue('zAuxNew')))

        # Set the goal position: PYTHON: goalPosition, MATLAB: zGoal  (size 2 x n)
        session.putvalue('zGoalNew',np.float_(goalPosition[roboName]))
        session.run('zGoal=[zGoal zGoalNew];')
        logging.info('Set goalPosition completed')
        logging.debug("in python: " + str(np.float_(goalPosition[roboName])))
        logging.debug("in MATLAB: " + str(session.getvalue('zGoalNew')))

        # Set the goal velocity: PYTHON: goalVelocity, MATLAB: vGoal  (size 2 x n)
        session.putvalue('vGoalNew',np.float_(goalVelocity[roboName]))
        session.run('vGoal=[vGoal vGoalNew];')
        logging.info('Set goalVelocity completed')
        logging.debug("in python: " + str(np.float_(goalVelocity[roboName])))
        logging.debug("in MATLAB: " + str(session.getvalue('vGoalNew')))

    # Convert everything into n-dim cell arrays
    session.run('zAux = mat2cell(zAux,size(zAux,1),ones(1,size(zAux,2)));')
    session.run('zGoal = mat2cell(zGoal,size(zGoal,1),ones(1,size(zGoal,2)));')
    session.run('vGoal = mat2cell(vGoal,size(vGoal,1),ones(1,size(vGoal,2)));')

    # Execute one step of the local planner and collect velocity components
    session.run('eval(executeLocalPlanner)')

    session.run('vOut_x = vHColFreeController(:,1);')
    session.run('vOut_y = vHColFreeController(:,2);')
    logging.debug('vx = ' + str(session.getvalue('vOut_x')))
    logging.debug('vy = ' + str(session.getvalue('vOut_y')))

    vx = session.getvalue('vOut_x')
    vy = session.getvalue('vOut_y')

    # return velocities
    return vx, vy

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session