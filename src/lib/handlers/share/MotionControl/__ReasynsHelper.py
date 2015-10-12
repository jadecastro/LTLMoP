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

system = platform.system()
pathToMatlabLocalPlanner = '/home/jon/Dropbox/Research/LowLevelControllerSynthesis/reasyns/scripts'

def initializeController(session, regions, scalingPixelsToMeters, limitsMap):
    """
    intialize Local Planner by setting up a pymatlab session and setting variables
    """
    logging.info('Starting Matlab session...')
    # session = pymatlab.session_factory()

    # Initialize the local planner
    session.run('cd '+pathToMatlabLocalPlanner)

    session.run('initExecute();')
    print str(session.getvalue('errorMsg'))

    session.run('[vx,vy,w,errorMsg,acLastData] = executeControllersSingleStep(aut,sys,ac_trans,ac_inward,pose,[],id_region_1,id_region_2,acLastData);')
    print str(session.getvalue('errorMsg'))

    # session.putvalue('rRob',robotRadius)
    # logging.debug("  rRob (matlab): "+str(session.getvalue('rRob')))

    session.putvalue('limitsMap',limitsMap)
    logging.debug("  limitsMap (matlab): "+str(session.getvalue('limitsMap')))

    
    # return matlab session
    # return session

def executeController(session, poseDic, regions, curr, next, coordmap_lab2map, scalingPixelsToMeters, doUpdate):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """

    numRobots = len(poseDic)

    for i, poseLoc in enumerate(poseDic.iteritems()):
        roboName = poseLoc[0]

        # Set the current pose: PYTHON: pose, MATLAB: zAux  (size d x n)  
        #pose = np.float_(np.hstack([float(1)/scalingPixelsToMeters*np.array(coordmap_lab2map(poseLoc[1][0:2])), poseLoc[1][2]]))
        pose = np.float_(np.hstack([float(1)/scalingPixelsToMeters*poseLoc[1][0:2], poseLoc[1][2]]))
        # poseNew = np.float_(poseLoc[1])
        session.putvalue('pose',pose)
        logging.debug("  pose (matlab): "+str(session.getvalue('pose')))
        # print("  pose (matlab): "+str(session.getvalue('pose')))

        if doUpdate[roboName]:

            currRegName = regions[curr[roboName]].name
            nextRegName = regions[next[roboName]].name
            print "current region: "+currRegName
            print "next region: "+nextRegName
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
            session.run('currentNextRegions('+str(i+1)+',:) = [id_region_1, id_region_2];')

            logging.debug("  id_region_1 (matlab): "+str(session.getvalue('id_region_1')))
            logging.debug("  id_region_2 (matlab): "+str(session.getvalue('id_region_2')))
            print "  id_region_1 (matlab): "+str(session.getvalue('id_region_1'))
            print "  id_region_2 (matlab): "+str(session.getvalue('id_region_2'))


    # Execute one step of the local planner and collect velocity components
    try:
        session.run('[vx,vy,w,errorMsg,acLastData] = executeControllersSingleStep(aut,sys,ac_trans,ac_inward,pose,[],id_region_1,id_region_2,acLastData);')
    except:
        print "WARNING: Matlab execute function experienced an error!!"

    logging.debug("  Matlab error: "+str(session.getvalue('errorMsg')))
    # print str(session.getvalue('errorMsg'))
    vx = {}
    vy = {}
    w = {}
    for i, poseLoc in enumerate(poseDic.iteritems()):
        # logging.debug('v = ' + str(session.getvalue('vOut'+str(i+1))))
        # logging.debug('w = ' + str(session.getvalue('wOut'+str(i+1))))

        vx[i] = 1*scalingPixelsToMeters*session.getvalue('vx')
        vy[i] = 1*scalingPixelsToMeters*session.getvalue('vy')
        w[i] = 1*session.getvalue('w') #0.25*session.getvalue('wOut'+str(i+1))
        # deadAgent.append(session.getvalue('deadlockAgent'+str(i+1)))
        # print "Deadlock status (agent "+str(i)+") :"+str(deadAgent[i])


    # return velocities
    return vx, vy, w

def closeInterface(session):
    """
    close connection with MATLAB
    """
    logging.info('MATLAB session closed')
    del session
