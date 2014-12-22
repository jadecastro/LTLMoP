import pymatlab
import numpy as np
import logging
import platform
from collections import OrderedDict
from math import sin, cos

threshold = 10
robRadius = OrderedDict([('rob2',0.15), ('rob1',0.15)])
robMaxVel = OrderedDict([('rob2',0.5), ('rob1',0.5)])
# regionNumbers = OrderedDict([('p9',1),('p7',2),('p5',3),('p6',4),('p8',5)])  # Hack: Temporarily hard-coding TODO: bring in from the region file.
robots = [robRadius, robMaxVel]
# This is a hack assuming javier uses Mac and Jonathan Windows or Linux
system = platform.system()
if system == 'Darwin':
    pathToMatlabLocalPlanner = '/Users/jalonso/MIT/drl-projects/uav-avoidance/multiquad-sim'
else:
    pathToMatlabLocalPlanner = '/home/jon/Dropbox/Repos/uav-avoidance/multiquad-sim'

def initializeLocalPlanner(regions, regionTransitionFaces, obstaclePoints, scalingPixelsToMeters, limitsMap):
    """
    intialize Local Planner by setting up a pymatlab session and setting variables
    """
    logging.info('Starting Matlab session...')
    session = pymatlab.session_factory()

    # Initialize the local planner
    session.run('cd '+pathToMatlabLocalPlanner)
    session.run('settingsHadas = 1;')
    session.run('simLocalPlanning_initialize();')

    # Set robot parameters for use in the current Matlab session
    rRadius = []
    for roboName, propRadius in robRadius.iteritems():
        rRadius.append([propRadius])
    robotRadius = np.float_(rRadius)

    # session.putvalue('rRob',robotRadius)
    # logging.debug("  rRob (matlab): "+str(session.getvalue('rRob')))

    session.putvalue('limitsMap',limitsMap)
    logging.debug("  limitsMap (matlab): "+str(session.getvalue('limitsMap')))

    session.putvalue('regionTransitionFaces',regionTransitionFaces)
    logging.debug("  regionTransitionFaces (matlab): "+str(session.getvalue('regionTransitionFaces')))
    session.run('sizeOut = size(regionTransitionFaces);')

    for i, points in enumerate(obstaclePoints):
        session.putvalue('obstaclePointsNew',points)
        logging.debug("  obstaclePointsNew (matlab): "+str(session.getvalue('obstaclePointsNew')))
        session.run('obstaclePoints{'+str(i+1)+'} = obstaclePointsNew')

    # Set the region/obstacle vertices
    session.run('[obstacle_rel, wallConstraintsXYZ, region_doors] = create_map_3d_from_2d_input(limitsMap, obstaclePoints, regionTransitionFaces);')
    session.run('initializeAgentParameters();')
    session.run('view(2);')
    session.run('parameters.n_dynamicObstacle = 1;')

    print "obstacle_rel: "+str(session.getvalue('obstacle_rel'))
    print "wallConstraintsXYZ: "+str(session.getvalue('wallConstraintsXYZ'))
    
    # return matlab session
    return session

def executeLocalPlanner(session, poseDic, goalPosition, goalVelocity, doUpdate, regions, curr, next, coordmap_lab2map, scalingPixelsToMeters):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """
    for i, poseLoc in enumerate(poseDic.iteritems()):
        roboName = poseLoc[0]

        # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)  
        # poseNew = np.float_(np.hstack([float(1)/scalingPixelsToMeters*np.array(coordmap_lab2map(poseLoc[1][0:2])), poseLoc[1][2]]))
        poseNew = np.float_(np.hstack([float(1)/scalingPixelsToMeters*poseLoc[1][0:2], poseLoc[1][2]]))
        # poseNew = np.float_(poseLoc[1])
        session.putvalue('poseNew'+str(i+1),poseNew)
        logging.debug("  poseNew"+str(i+1)+" (matlab): "+str(session.getvalue('poseNew'+str(i+1))))

        if doUpdate[roboName]:
            # Set the goal position: PYTHON: goalPosition, MATLAB: zGoal  (size 2 x n)
            # session.putvalue('zGoalNew'+str(i+1),float(1)/scalingPixelsToMeters*np.float_(np.array(coordmap_lab2map(goalPosition[roboName]))))
            session.putvalue('zGoalNew'+str(i+1),float(1)/scalingPixelsToMeters*np.float_(goalPosition[roboName]))
            # logging.debug("  zGoalNew"+str(i+1)+" (matlab): "+str(session.getvalue('zGoalNew'+str(i+1))))

            currRegName = regions[curr[roboName]].name
            nextRegName = regions[next[roboName]].name
            print "current region: ",currRegName
            print "next region: ",nextRegName
            # print regionNumbers
            currNbr = [[]]; nextNbr =[[]]
            for currRegNbr, region in enumerate(regions):
                if region.name == regions[curr[roboName]].name:
                    break
            for nextRegNbr, region in enumerate(regions):
                if region.name == regions[next[roboName]].name:
                    break
            print currRegNbr, nextRegNbr
            currNbr[0] = currRegNbr+1; nextNbr[0] = nextRegNbr+1
            # currRegNbr[0] = regions.name.index(currRegName) #regionNumbers[currRegName]
            # nextRegNbr[0] = regionNumbers[nextRegName]
            session.putvalue('id_region_1',np.float_(currNbr))
            session.putvalue('id_region_2',np.float_(nextNbr))
            # logging.debug("  id_region_1 (matlab): "+str(session.getvalue('id_region_1')))
            # logging.debug("  id_region_2 (matlab): "+str(session.getvalue('id_region_2')))
            session.run('allowed_regions('+str(i+1)+',:) = [id_region_1, id_region_2];')

            print 'robot '+str(i)+', curr:'+str(currNbr)
            print 'robot '+str(i)+', next:'+str(nextNbr)
    # Execute one step of the local planner and collect velocity components
    session.run('simLocalPlanning_doStep_wrapper();')

    v = {}
    w = {}
    deadAgent = {}
    for i, poseLoc in enumerate(poseDic.iteritems()):
        # logging.debug('v = ' + str(session.getvalue('vOut'+str(i+1))))
        # logging.debug('w = ' + str(session.getvalue('wOut'+str(i+1))))

        v[i] = 1*scalingPixelsToMeters*session.getvalue('vOut'+str(i+1))
        w[i] = session.getvalue('wOut'+str(i+1))
        deadAgent[i] = session.getvalue('deadlockAgent'+str(i+1))
        # print "Deadlock status (agent "+str(i)+") :"+str(deadAgent[i])

    if any(deadAgent):
        # update slugsin with current state of the unmoving robots and unmoving dynamic obstacles
        # save the current poses, and regions.
        # TODO: keep the strategy running while resynthesis is happening

        pass

    # return velocities
    return v, w

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session
