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

def initializeLocalPlanner(session, regions, regionTransitionFaces, obstaclePoints, scalingPixelsToMeters, limitsMap, numRobots, numDynamicObstacles, numExogenousRobots, robotType, scenario):
    """
    intialize Local Planner by setting up a pymatlab session and setting variables
    """
    logging.info('Starting Matlab session...')
    # session = pymatlab.session_factory()

    # Initialize the local planner
    session.run('cd '+pathToMatlabLocalPlanner)
    if scenario == 1:
        session.run('settingsHadas = 1;')
        session.run('isFirstCall = 1;')
    elif scenario == 2:
        session.run('settingsHadas = 2;')

    session.run('nB = '+str(numRobots+numExogenousRobots+numDynamicObstacles)+';')
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
        session.run('obstaclePoints{'+str(i+1)+'} = obstaclePointsNew;')

    # Set the region/obstacle vertices
    if scenario == 1:
        session.run('[obstacle_rel, wallConstraintsXYZ, region_doors] = create_map_3d_from_2d_input(limitsMap, obstaclePoints, regionTransitionFaces);')
    elif scenario == 2:
        session.run('[obstacle_rel, wallConstraintsXYZ, region_doors] = create_map_3d(scenario_type);')

    for i in range(numRobots):
        session.run('agentType('+str(i+1)+') = '+str(robotType)+';')
        # put initial values into the variables we will be querying
        if i == 0:
            session.run('vOut'+str(i+1)+' = 1;')
            session.run('wOut'+str(i+1)+' = 2;')
        elif i == 1:
            session.run('vOut'+str(i+1)+' = 2;')
            session.run('wOut'+str(i+1)+' = 2;')
        session.run('deadlockAgent'+str(i+1)+' = 0;')

    session.run('initializeAgentParameters();')
    session.run('view(2);')
    session.run('parameters.n_dynamicObstacle = '+str(numDynamicObstacles)+';')
    if numDynamicObstacles > 0:
        session.run('parameters.dynamicObstacleVelocityControlled = 1;')

    # initially set the allowed regions to zero (no constraints)
    for i in range(numRobots+numExogenousRobots):
        session.run('allowed_regions('+str(i+1)+',:) = [0, 0];')

    # print "obstacle_rel: "+str(session.getvalue('obstacle_rel'))
    # print "wallConstraintsXYZ: "+str(session.getvalue('wallConstraintsXYZ'))

    
    # return matlab session
    # return session

def executeLocalPlanner(session, poseDic, goalPosition, goalVelocity, poseExog, goalPositionExog, goalVelocityExog, doUpdate, regions, curr, next, coordmap_lab2map, scalingPixelsToMeters, numDynamicObstacles, scenario):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """

    numRobots = len(poseDic)

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
            logging.debug("  zGoalNew"+str(i+1)+" (matlab): "+str(session.getvalue('zGoalNew'+str(i+1))))

            currRegName = regions[curr[roboName]].name
            nextRegName = regions[next[roboName]].name
            print "current region: "+currRegName
            print "next region: "+nextRegName
            print "  zGoalNew"+str(i+1)+" (matlab): "+str(session.getvalue('zGoalNew'+str(i+1)))
            # print regionNumbers
            currNbr = [[]]; nextNbr =[[]]
            for currRegNbr, region in enumerate(regions):
                if region.name == regions[curr[roboName]].name:
                    break
            for nextRegNbr, region in enumerate(regions):
                if region.name == regions[next[roboName]].name:
                    break
            # print currRegNbr, nextRegNbr
            currNbr[0] = currRegNbr+1; nextNbr[0] = nextRegNbr+1
            # currRegNbr[0] = regions.name.index(currRegName) #regionNumbers[currRegName]
            # nextRegNbr[0] = regionNumbers[nextRegName]
            session.putvalue('id_region_1',np.float_(currNbr))
            session.putvalue('id_region_2',np.float_(nextNbr))
            session.run('allowed_regions('+str(i+1)+',:) = [0., 0.];') #[id_region_1, id_region_2];')
            # logging.debug("  id_region_1 (matlab): "+str(session.getvalue('id_region_1')))
            # logging.debug("  id_region_2 (matlab): "+str(session.getvalue('id_region_2')))

            # print 'robot '+str(i)+', curr:'+str(currNbr)
            # print 'robot '+str(i)+', next:'+str(nextNbr)

    for i in range(len(poseExog)):
        # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)  
        # poseNew = np.float_(np.hstack([float(1)/scalingPixelsToMeters*np.array(coordmap_lab2map(poseLoc[1][0:2])), poseLoc[1][2]]))
        poseNew = np.float_(np.hstack([float(1)/scalingPixelsToMeters*(poseExog[i][0:2]), poseExog[i][2]]))
        # poseNew = np.float_(poseLoc[1])
        session.putvalue('poseNew'+str(numRobots+i+1),poseNew)
        # logging.debug("  poseNew"+str(numRobots+i+1)+" (matlab): "+str(session.getvalue('poseNew'+str(numRobots+i+1))))

        doUpdateExog = 1  # TODO: faster to bring in the flag every time step?

        if doUpdateExog:
            # Set the goal position: PYTHON: goalPosition, MATLAB: zGoal  (size 2 x n)
            # session.putvalue('zGoalNew'+str(i+1),float(1)/scalingPixelsToMeters*np.float_(np.array(coordmap_lab2map(goalPosition[roboName]))))
            session.putvalue('zGoalNew'+str(numRobots+i+1),float(1)/scalingPixelsToMeters*np.float_(goalPositionExog[i]))
            # logging.debug("  zGoalNew"+str(numRobots+i+1)+" (matlab): "+str(session.getvalue('zGoalNew'+str(numRobots+i+1))))
            # session.run('id_region_1 = allowed_regions('+str(numRobots+i+1)+',1); id_region_2 = allowed_regions('+str(numRobots+i+1)+',2);')
            # logging.debug("  id_region_1 (matlab): "+str(session.getvalue('id_region_1')))
            # logging.debug("  id_region_2 (matlab): "+str(session.getvalue('id_region_2')))


    # Execute one step of the local planner and collect velocity components
    try:
        session.run('doStep_wrapper();')
    except:
        print "WARNING: Matlab function doStep_wrapper experienced an error!!"

    v = {}
    w = {}
    deadAgent = []
    for i, poseLoc in enumerate(poseDic.iteritems()):
        # logging.debug('v = ' + str(session.getvalue('vOut'+str(i+1))))
        # logging.debug('w = ' + str(session.getvalue('wOut'+str(i+1))))

        v[i] = 1*scalingPixelsToMeters*session.getvalue('vOut'+str(i+1))
        w[i] = 1*session.getvalue('wOut'+str(i+1)) #0.25*session.getvalue('wOut'+str(i+1))
        # deadAgent.append(session.getvalue('deadlockAgent'+str(i+1)))
        # print "Deadlock status (agent "+str(i)+") :"+str(deadAgent[i])

    vd = {}
    wd = {}
    deadAgentD = []
    if numDynamicObstacles == 0:
        for i in range(len(poseExog)):
            vd[i] = 1*scalingPixelsToMeters*session.getvalue('vOut'+str(numRobots+i+1))
            wd[i] = session.getvalue('wOut'+str(numRobots+i+1))
            deadAgentD.append(session.getvalue('deadlockAgent'+str(numRobots+i+1)))

    # return velocities
    return v, w, vd, wd, deadAgent

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session
