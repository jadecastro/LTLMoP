import pymatlab
import numpy as np
import logging
from collections import OrderedDict
from math import sin, cos

threshold = 10
robRadius = OrderedDict([('rob2',0.15), ('rob1',0.15)])
robMaxVel = OrderedDict([('rob2',0.5), ('rob1',0.5)])
regionNumbers = OrderedDict([('B',1),('L',2),('T',3),('R',4),('D',5)])
robots = [robRadius, robMaxVel]
pathToMatlabLocalPlanner = '/home/jon/Dropbox/Repos/uav-avoidance/multiquad-sim'
# pathToMatlabLocalPlanner = '/Users/jalonso/MIT/drl-projects/uav-avoidance/multiquad-sim'

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

    session.run('cd '+pathToMatlabLocalPlanner)
    session.run('settingsHadas = 1;')
    session.run('simLocalPlanning_initialize();')
    session.run('view(2);')

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

def executeLocalPlanner(session, poseDic, goalPosition, goalVelocity, doUpdate, regions, curr, next):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """
    regionNumbers

    for i, poseLoc in enumerate(poseDic.iteritems()):
        roboName = poseLoc[0]

        # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)    
        session.putvalue('poseNew'+str(i+1),np.float_(poseLoc[1]))
        logging.info('Set robotPose completed')
        # logging.debug("  in python: " + str(np.float_(poseLoc[1])))
        #logging.debug("  in MATLAB: " + str(session.getvalue('poseNew'+str(i+1))))

        if doUpdate[roboName]:
            # Set the goal position: PYTHON: goalPosition, MATLAB: zGoal  (size 2 x n)
            session.putvalue('zGoalNew'+str(i+1),np.float_(goalPosition[roboName]))
            logging.info('Set goalPosition completed')

            curr_reg = regionNumbers[curr[roboName].name]
            next_reg = regionNumbers[next[roboName].name]

            currRegNbr = regions[curr_reg].name
            nextRegNbr = regions[next_reg].name
            trans = [currRegNbr, nextRegNbr]
            session.putvalue('id_region_1',np.float_(trans(0)))
            session.putvalue('id_region_2',np.float_(trans(1)))
            session.run('allowed_regions('+str(i+1)+',:) = [id_region_1, id_region_2];')
            # logging.debug("  in python: " + str(np.float_(goalPosition[roboName])))
            #logging.debug("  in MATLAB: " + str(session.getvalue('zGoalNew'+str(i+1))))

            # session.putvalue('vGoalNew'+str(i+1),np.float_(goalVelocity[roboName]))
            # logging.info('Set goalVelocity completed')
            # logging.debug("  in python: " + str(np.float_(goalVelocity[roboName])))
            # logging.debug("  in MATLAB: " + str(session.getvalue('vGoalNew')))

    # Execute one step of the local planner and collect velocity components
    session.run('simLocalPlanning_doStep_wrapper();')

    v = {}
    w = {}
    for i, poseLoc in enumerate(poseDic.iteritems()):
        # logging.debug('v = ' + str(session.getvalue('vOut')))
        # logging.debug('w = ' + str(session.getvalue('wOut')))
        v[i] = session.getvalue('vOut'+str(i+1))
        w[i] = session.getvalue('wOut'+str(i+1))

    # return velocities
    return v, w

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session
