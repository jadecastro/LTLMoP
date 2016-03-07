import numpy as np
import logging
import platform
from collections import OrderedDict
from math import sin, cos

import scipy.io as sio

system = platform.system()

def initializeController(fname, numDiscStates, numDiscTransitions):
    """
    intialize Local Planner by setting up a pymatlab session and setting variables
    """

    mat_contents = sio.loadmat(fname)

    t_base = 0
    acLastData = [1, 0, 5, 1, [], []]

    numInward = len(mat_contents['ac_inward'][0])
    numtrans = len(mat_contents['ac_trans'][0])
    #if (not len(mat_contents['ac_inward'][0]) == numDiscStates):
        #TODO: throw an error!

    #unpack the mat file contents
    aut = mat_contents['aut'][0]

    ac_inward = [[]]*numInward
    for i in range(numInward):
        t = mat_contents['ac_inward'][0][i]['t'][0][0]
        x0 = mat_contents['ac_inward'][0][i]['x0'][0][0]
        u0 = mat_contents['ac_inward'][0][i]['u0'][0][0]
        K = mat_contents['ac_inward'][0][i]['P'][0][0]
        #Einv = mat_contents['ac_inward'][0][i]['Einv'][0][0]['P']
        rho = mat_contents['ac_inward'][0][i]['rho'][0][0]
        P = mat_contents['ac_inward'][0][i]['P'][0][0]

        ac_inward[i] = {'t':t, 'x0':x0, 'u0':u0, 'K':K, 'rho':,rho, 'P':,P}

    ac_trans = [[]]*numTrans
    for i in range(numTrans):
        t = mat_contents['ac_trans'][0][i]['t'][0][0]
        x0 = mat_contents['ac_trans'][0][i]['x0'][0][0]
        u0 = mat_contents['ac_trans'][0][i]['u0'][0][0]
        K = mat_contents['ac_trans'][0][i]['P'][0][0]
        #Einv = mat_contents['ac_trans'][0][i]['Einv'][0][0]['P']
        rho = mat_contents['ac_trans'][0][i]['rho'][0][0]
        P = mat_contents['ac_trans'][0][i]['P'][0][0]

        ac_trans[i] = {'t':t, 'x0':x0, 'u0':u0, 'K':K, 'rho':,rho, 'P':,P}

    # pre-compute a trinary vector for making complete funnel-containment checks
    numCyclicStates = len(sysObj.isCyclic.nonzero())
    vec, cyclicTrinaryVector = buildTrinary([],[[]],numCyclicStates)

    # make [0, ..., 0] the first element to try
    indexToPop = cyclicTrinaryVector.index([0]*numCyclicStates)
    cyclicTrinaryVector = [[0]*numCyclicStates, cyclicTrinaryVector.pop(indexToPop)]
    
    return aut ac_inward ac_inward acLastData cyclicTrinaryVector

def executeController(session, poseDic, regions, curr, next, coordmap_lab2map, scalingPixelsToMeters, doUpdate):
    """
    pose  = {'rob1':[-1 ,.5],'rob2':[1,1],'rob3':[3.5 , -1]}
    next_regIndices = {'rob1': 2,'rob2':3,'rob3':3}
    """

    numRobots = len(poseDic)

    for i, poseLoc in enumerate(poseDic.iteritems()):
        roboName = poseLoc[0]

        #pose = np.float_(np.hstack([float(1)/scalingPixelsToMeters*np.array(coordmap_lab2map(poseLoc[1][0:2])), poseLoc[1][2]]))
        pose = np.float_(np.hstack([float(1)/scalingPixelsToMeters*poseLoc[1][0:2], poseLoc[1][2]]))

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

            print "  currRegNbr: "+str(currRegNbr)
            print "  nextRegNbr: "+str(nextRegNbr)


    # Execute one step of the local planner and collect velocity components
    try:
        vx,vy,w,acLastData,data = executeSingleStep(aut,ac_trans,ac_inward,pose,currRegNbr,nextRegNbr,acLastData)
    except:
        print "WARNING: Matlab execute function experienced an error!!"

    Vx = {}
    Vy = {}
    W = {}
    for i, poseLoc in enumerate(poseDic.iteritems()):
        # logging.debug('v = ' + str(session.getvalue('vOut'+str(i+1))))
        # logging.debug('w = ' + str(session.getvalue('wOut'+str(i+1))))

        Vx[i] = 1*scalingPixelsToMeters*vx
        Vy[i] = 1*scalingPixelsToMeters*vy
        W[i] = 1*w #0.25*session.getvalue('wOut'+str(i+1))
        # deadAgent.append(session.getvalue('deadlockAgent'+str(i+1)))
        # print "Deadlock status (agent "+str(i)+") :"+str(deadAgent[i])


    # return velocities
    return Vx, Vy, W


def executeSingleStep(aut,ac_trans,ac_inward,pose,x,t,currReg,nextReg,acLastData,data):
    t_offset = data['t_offset']
    t_base = data['t_base']
    tend = data['tend']
    ell = data['ell']
    t_trials = data['t_trials']
    t_sav = data['t_sav']
    x_sav = data['x_sav']
    x0_sav = data['x0_sav']

    delayTime = -0.1

    timeBaseFlag = False  # if 'true', use time as a basis for choosing the TVLQR states; otherwise use a weighted Euclidean distance.
    useLastAC = True

    phaseWrap = np.array([0, 0, 2*pi])

    weights = np.array([1, 1, 0.2])

    m = sysObj.numConfig
    n = sysObj.numState

    #################
    # Account for our manual shifting of the map to the left because of the "dead zone" in the Vicon field 
    x = x + np.array([0.0043, 0.1629, 0])
    #################

    #################
    # Account for our manual shifting of the map
    x = x + np.array([-0.9774, 0.0258, 0])
    #################

    x[2] = x[2] + 0.2  # theta bias needed to account for misalignment of the youBot's coordinate frame wrt. its true orientation

    prevCtrl = False

    if not t_base:
        t_base = clock #FIX

    # fail-safe velocity settings
    vx = 0
    vy = 0
    w = 0.001

    teval = 0.02;

    # flip! if necessary
    #if size(x,2) > size(x,1):
    #    x = x'
    
    trans = aut['trans']
    q = aut['q']

    currStateVec = (q == currReg).nonzero()
    nextStateVec = (q == nextReg).nonzero()
    currTrans = ((currStateVec in trans[0]) and (nextStateVec in trans[1])).nonzero()
    if not currTrans:
        print 'you have specified an invalid transition!'
    currState = trans[0,currTrans]
    nextState = trans[1,currTrans]
    
    # identify the funnel we're in and get the ellipse index
    iTrans = False
    iIn = False
    if isinternalUnion(ac_trans[currTrans], x, cyclicTrinaryVector):
    # if isinternal(ac_trans{currTrans}, x,'u')
        iTrans = currTrans
        ac = ac_trans[currTrans]
    if (iTrans == False) and not ac_inward:
        for funIdx = 1 in range(len(ac_inward[currState])):
            if isinternalUnion(ac_inward[currState][funIdx], x, cyclicTrinaryVector):
                #         if isinternal(ac_inward{currState}, x,'u')
                iIn = currState
                ac = ac_inward[currState][funIdx]
    
    # if no funnels in the new region/transition OR have temporarily left a funnel, then keep activating the previous one.
    currStateOld = []
    nextStateOld = []
    if not iTrans and not iIn:
        if useLastAC:
            # propagate the appropriate old states on to the next iteration
            if not isempty(acLastData[4]) and not isempty(acLastData[5]):
                currStateOld = acLastData[4]
                nextStateOld = acLastData[5]
            else:
                currStateOld = acLastData[2]
                nextStateOld = acLastData[3]

            currTransOld = ( trans[0] == currStateOld ) and ( trans[1] == nextStateOld )
            if not acLastData[0]:
                ac = ac_trans[currTransOld]
            else:
                ac = ac_inward[currStateOld]
            iTrans = acLastData[0]
            iIn = acLastData[1]
            prevCtrl = True
            print('WARNING: no funnels found! Using the previous controller.')
        else:
            iTrans = currTrans
            ac = ac_trans[currTrans]
            print('WARNING: no funnels found! Forcing an (unverified) controller.')
    
    # determine the new time offset if something has changed
    # TODO: do something smarter here: 
    #       we can remove all the times in the current trajectory prior to the current time, and also possibly add time-weighting to the Euclidean fit
    if timeBaseFlag:  
        # TODO: This option is work-in-progress
        
        # acData = {iTrans, iIn, currState, nextState, currStateOld, nextStateOld};
        
        # for i in range(len(acData)-2):
        #     cmp(i) = acData{i}==acLastData{i}
        # cmp
        # if (~all(cmp) || ~(all(ismember([acLastData{1:4}],[acData{1:4}])) && all([acLastData{3:4}] == [acData{3:4}]))) && ~prevCtrl
        #     t_trials = getTimeVec(ac.x0)
        #     tend = t_trials(end)
        #     t_base = clock
        #     ell = ellipsoid(ac)
            
        #     for i = 1:length(ell)
        #         if isinternal(ell(i),x,'u')
        #             t_offset = t_trials(i)
        #             break

        # acLastData = acData
        
        # if isempty(t):  # if time is not given, we compute it here
        #     t = etime(clock,t_base) + delayTime
        # t_offset
        # teval = min(tend,t + t_offset)

    else:  
        # This approach computes the command based on a weighted Euclidean distance to the nominal trajectory

        acData = [iTrans, iIn, currState, nextState, currStateOld, nextStateOld]
        
        for i in range(len(acData)-2):
            cmp[i] = acData[i]==acLastData[i]
        if not all(cmp) or not (all(ismember([acLastData[0:4]],[acData[0:4]])) and all([acLastData[2:4]] == [acData[2:4]])):
            # a change has been detected!!
            t_base = clock
            t_trials = ac['t']

        acLastData = acData
        
        minDelta = np.inf
        for i in range(len(t_trials)):
            xtmp = double(ac['x0'], t_trials[i])
            testDist = np.array([
                np.linalg.norm(weights*(sysObj.state2SEconfig(xtmp) - (sysObj.state2SEconfig(x) + phaseWrap))),
                np.linalg.norm(weights*(sysObj.state2SEconfig(xtmp) - sysObj.state2SEconfig(x)),
                np.linalg.norm(weights*(sysObj.state2SEconfig(xtmp) - (sysObj.state2SEconfig(x) - phaseWrap)))])

            testMinDist = min(testDist)

            if (testMinDist < minDelta):
                teval = t_trials[i]
                minDelta = testMinDist
        if not t:  # if time is not given, we compute it here
            t = etime(clock,t_base)#FIX
    
    # compute a command
    teval
    K = double(ac['K'],teval) 
    x0 = double(ac['x0'],teval) 
    u0 = double(ac['u0'],teval)
    
    K = K(end-length(u0)+1:end,:)
    
    u_test = np.array([
            (K*(x - x0 + sysObj.state2SEconfig(phaseWrap))),
            (K*(x - x0)),
            (K*(x - x0 - sysObj.state2SEconfig(phaseWrap)))])
    u_idx = (abs(u_test[:,-1]) == min(abs(u_test[:,-1]))).nonzero()
    u_ctrl = u_test[u_idx[0]]

    # Compute the control command
    u = u0 + u_ctrl
    # u(2) = max(min(u(2),16),-16);
    
    # Allocate the linear/angular velocities
    vx, vy, w = sysObj.command2robotInput(x,u)
    
    #u_sav = [u_sav; t u]
    t_sav = np.append(t_sav, t)
    x_sav = np.append(x_sav, x)
    x0_sav = np.append(x0_sav, x0)

    data = {'t_offset': t_offset,'t_base': t_base,'tend': tend,'ell': ell,'t_trials': t_trials,'t_sav': t_sav,'x_sav': x_sav,'x0_sav': x0_sav}

    return vx, vy, w, acLastData, data

def isinternalUnion(ac, xTest, cyclicTrinaryVector):

    for vec in cyclicTrinaryVector:
        idx = 0
        xTest = x
        for j in sysObj.isCyclic:
            xTest[:][j] = x[:][j] + vec[idx] * 2*np.pi
            idx += 1

        res = checkSingleState(ac, xTest)
        if res:
            return 1
    return 0

def buildTrinary(vec, dvec, size):

    if len(vec) == size:
        dvec.append(vec[:])
        return vec, dvec

    else:
        vec.append(-1)
        #print vec
        for j in range(-1,2): 
            vec[-1] = j
            # print vec
            vec, dvec = buildTrinary(vec, dvec, size)

        return vec[:-1], dvec

def checkSingleState(ac, x):

    for i in range(x.shape[1]):
        q = x[:,i] - ac['x0']
        Qinv = ac['P']
        
        # Assume Qinv is not ill-conditioned
        r = q * (Qinv * q)
        
        if (r < 1.) or (abs(r - 1.) < 1e-6):
            #if all(eig(Qinv) >= -1e-6):
            return 1
    return 0

def double():

