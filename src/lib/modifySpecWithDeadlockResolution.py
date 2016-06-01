#!/usr/bin/python

import sys
import StringIO
import re
from collections import OrderedDict

# TODO: get this from the spec
NUM_ROBOTS = 2

def parseRegionData(regData):
    """
    Parse a region file, extracting lists of face and region data
    """

    modeTrans = 0
    faces = []
    for line in regData.readlines():
        line = line.strip()
        if line == "":
            pass
        elif line.startswith("Transitions:"):
            modeTrans = 1
        else:
            if modeTrans == 1:
                line = line.split('\t')
                faces.append(line)
    regData.seek(0)

    modeTrans = 0
    regions = []
    lastLine = ''
    thisType = ['']
    thisPoint = []
    thisSize = []
    thisPosition = []
    for line in regData.readlines():
        # print line
        # print line.find("name")
        line = line.strip()
        if line == "":
            pass
        elif line.find("\"name\":") != -1:
            line = line.strip("\"name\":")
            thisName = re.findall("[a-zA-Z]+", line)[0]
            print thisName
        elif line.find("\"type\":") != -1:
            line = line.strip("\"type\":")
            thisType = re.findall("[a-zA-Z]+", line)[0]
            # print thisType

        if line == "":
            pass
        elif line.find("\"points\":") != -1:
            modeTrans = 1
        elif line.find("\"size\":") != -1:
            modeTrans = 2
        elif line.find("\"position\":") != -1:
            modeTrans = 3
        elif re.findall("[a-zA-Z]+", line) != []:
            modeTrans = 0
        elif line.find("}") != -1:
            # thisPoint = filter(None, thisPoint) 
            thisPoint = zip(*[iter(thisPoint)]*2)
            # print thisPoint
            thisSize = zip(*[iter(thisSize)]*2)
            # print thisSize
            # thisPosition = filter(None, thisPosition) 
            thisPosition = zip(*[iter(thisPosition)]*2)
            # print thisPosition
            if thisType[0] == 'rect':
                pass

            regions.append({"name": thisName,
                           "type": thisType,
                           "position": thisPosition,
                           "points": thisPoint,
                           "size": thisSize})

            thisPoint = []
            thisSize = []
            thisPosition = []
            thisType = ['']
        else:
            lastLine = line;
            tmp = re.findall("\d+.\d+",line)
            # if there is a number here, extract it
            if len(tmp) > 0:
                if modeTrans == 1:
                    thisPoint.append(tmp[0])
                if modeTrans == 2:
                    thisSize.append(tmp[0])
                elif modeTrans == 3:
                    thisPosition.append(tmp[0])

    return faces, regions


def findPairwiseDeadlockConditions(regions,faces,robNumA,robNumB):
    """

    """
    # (rob1_T_rc' & (rob2_B_rc' | rob2_D_rc')) -> (!rob12_deadlock')

    addedEnvironmentSafetyStatement = []
    for region in regions:
        if region['name'] == 'boundary':
            pass
        else:

            regA = region['name']
            newStatement = "| ! & rob"+str(robNumA)+"_"+regA+"_rc' ! "

            # Start by populating the '|'
            for face in faces:
                if (face[0] == regA and face[1] != 'boundary') or (face[1] == regA and face[0] != 'boundary'):
                    newStatement += "| "

            newStatement += "rob"+str(robNumB)+"_"+regA+"_rc' "

            for face in faces:
                if (face[0] == regA and face[1] != 'boundary') or (face[1] == regA and face[0] != 'boundary'):
                    if regA == face[0]:
                        regB = face[1]
                    else:
                        regB = face[0]

                    newStatement += "rob"+str(robNumB)+"_"+regB+"_rc' "
            
            newStatement += "! rob"+str(robNumA)+str(robNumB)+"_deadlock'"
            addedEnvironmentSafetyStatement.append(newStatement)
            # print newStatement                            

    return addedEnvironmentSafetyStatement


def findDisabledTransitionConditions(regions,faces,robNumA):
    """

    """
    # # disable the current transition for the entirety of the duration in that region
    # #(m_rob1_deadlock1 & rob1_T_rc) -> (!rob1_T')
    # #(m_rob1_deadlock2 & rob1_T_rc) -> (!rob1_L' & !rob1_T')
    # (m_rob1_deadlock3 & rob1_T_rc) -> (!rob1_R' & !rob1_T')
    # (!m_rob1_deadlock1) -> (((!rob1_deadlock & rob1_deadlock') & (rob1_T_rc & rob1_T_rc' & rob1_T)) -> m_rob1_deadlock1')
    # (!m_rob1_deadlock2) -> (((!rob1_deadlock & rob1_deadlock') & (rob1_T_rc & rob1_T_rc' & rob1_L)) -> m_rob1_deadlock2')
    # (!m_rob1_deadlock3) -> (((!rob1_deadlock & rob1_deadlock') & (rob1_T_rc & rob1_T_rc' & rob1_R)) -> m_rob1_deadlock3')
    # ((m_rob1_deadlock1) & (rob1_T_rc & rob1_T_rc')) -> (((rob1_T_rc & rob1_T_rc')) <-> m_rob1_deadlock1')
    # ((m_rob1_deadlock2) & (rob1_T_rc & rob1_T_rc')) -> (((rob1_T_rc & rob1_T_rc')) <-> m_rob1_deadlock2')
    # ((m_rob1_deadlock3) & (rob1_T_rc & rob1_T_rc')) -> (((rob1_T_rc & rob1_T_rc')) <-> m_rob1_deadlock3')

    addedSystemPropositions = []
    addedSystemSafetyStatement = []
    addedSystemInitialConditions = []

    numNeededDeadlockMempropsPerRobot = 1

    for region in regions:
        if region['name'] == 'boundary':
            pass
        else:

            regA = region['name']

            # the first deadlock memory proposition is reserved for preventing the robot from remaining in the same region
            addedSystemSafetyStatement.append("| ! & m_rob"+str(robNumA)+"_deadlock1 rob"+str(robNumA)+"_"+regA+"_rc ! rob"+str(robNumA)+"_"+regA+"'")
            addedSystemSafetyStatement.append("| m_rob"+str(robNumA)+"_deadlock1 | ! & & & & ! rob"+str(robNumA)+"_deadlock rob"+str(robNumA)+"_deadlock' rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' rob"+str(robNumA)+"_"+regA+" m_rob"+str(robNumA)+"_deadlock1'")
            addedSystemSafetyStatement.append("| ! & & m_rob"+str(robNumA)+"_deadlock1 rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' | ! & rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' m_rob"+str(robNumA)+"_deadlock1'")
            addedSystemSafetyStatement.append("| ! & & m_rob"+str(robNumA)+"_deadlock1 rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' | ! m_rob"+str(robNumA)+"_deadlock1' & rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc'")

            count = 1
            for face in faces:
                if (face[0] == regA and face[1] != 'boundary') or (face[1] == regA and face[0] != 'boundary'):

                    count += 1

                    if regA == face[0]:
                        regB = face[1]
                    else:
                        regB = face[0]

                    addedSystemSafetyStatement.append("| ! & m_rob"+str(robNumA)+"_deadlock"+str(count)+" rob"+str(robNumA)+"_"+regA+"_rc & ! rob"+str(robNumA)+"_"+regA+"' ! rob"+str(robNumA)+"_"+regB+"'")
                    addedSystemSafetyStatement.append("| m_rob"+str(robNumA)+"_deadlock"+str(count)+" | ! & & & & ! rob"+str(robNumA)+"_deadlock rob"+str(robNumA)+"_deadlock' rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' rob"+str(robNumA)+"_"+regB+" m_rob"+str(robNumA)+"_deadlock"+str(count)+"'")
                    addedSystemSafetyStatement.append("| ! & & m_rob"+str(robNumA)+"_deadlock"+str(count)+" rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' | ! & rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' m_rob"+str(robNumA)+"_deadlock"+str(count)+"'")
                    addedSystemSafetyStatement.append("| ! & & m_rob"+str(robNumA)+"_deadlock"+str(count)+" rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' | ! m_rob"+str(robNumA)+"_deadlock"+str(count)+"' & rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc'")                        

                    numNeededDeadlockMempropsPerRobot = max(numNeededDeadlockMempropsPerRobot, count)

    for i in range(1,numNeededDeadlockMempropsPerRobot+1):

        addedSystemPropositions.append("m_rob"+str(robNumA)+"_deadlock"+str(i))
        addedSystemInitialConditions.append("! m_rob"+str(robNumA)+"_deadlock"+str(i))
        
    return addedSystemSafetyStatement, addedSystemPropositions, addedSystemInitialConditions, numNeededDeadlockMempropsPerRobot


def findPairwiseCoordinationConditions(regions,faces,robNumA,robNumB):
    """

    """
    # # coordination - what happens when two robots deadlock at the same time?
    # (!rob12_deadlock & rob12_deadlock') -> ((
        #((!m_rob1_deadlock1 & rob1_T_rc & rob1_T_rc' & rob1_T) -> m_rob1_deadlock1') & 
        #((!m_rob1_deadlock2 & rob1_T_rc & rob1_T_rc' & rob1_L) -> m_rob1_deadlock2') & 
        #((!m_rob1_deadlock3 & rob1_T_rc & rob1_T_rc' & rob1_R) -> m_rob1_deadlock3') & 
        #((!m_rob1_deadlock1 & rob1_B_rc & rob1_B_rc' & rob1_B) -> m_rob1_deadlock1') & 
        #((!m_rob1_deadlock2 & rob1_B_rc & rob1_B_rc' & rob1_L) -> m_rob1_deadlock2') & 
        #((!m_rob1_deadlock3 & rob1_B_rc & rob1_B_rc' & rob1_D) -> m_rob1_deadlock3') & 
        #((!m_rob1_deadlock1 & rob1_L_rc & rob1_L_rc' & rob1_L) -> m_rob1_deadlock1') & 
        #((!m_rob1_deadlock2 & rob1_L_rc & rob1_L_rc' & rob1_B) -> m_rob1_deadlock2') & 
        #((!m_rob1_deadlock3 & rob1_L_rc & rob1_L_rc' & rob1_T) -> m_rob1_deadlock3') & 
        #((!m_rob1_deadlock1 & rob1_R_rc & rob1_R_rc' & rob1_R) -> m_rob1_deadlock1') & 
        #((!m_rob1_deadlock2 & rob1_R_rc & rob1_R_rc' & rob1_D) -> m_rob1_deadlock2') & 
        #((!m_rob1_deadlock3 & rob1_R_rc & rob1_R_rc' & rob1_T) -> m_rob1_deadlock3') & 
        #((!m_rob1_deadlock1 & rob1_D_rc & rob1_D_rc' & rob1_D) -> m_rob1_deadlock1') & 
        #((!m_rob1_deadlock2 & rob1_D_rc & rob1_D_rc' & rob1_B) -> m_rob1_deadlock2') & 
        #((!m_rob1_deadlock3 & rob1_D_rc & rob1_D_rc' & rob1_R) -> m_rob1_deadlock3')) | 
    #(((!m_rob2_deadlock1 & rob2_T_rc & rob2_T_rc' & rob2_T) -> m_rob2_deadlock1') & 
        #((!m_rob2_deadlock2 & rob2_T_rc & rob2_T_rc' & rob2_L) -> m_rob2_deadlock2') & 
        #((!m_rob2_deadlock3 & rob2_T_rc & rob2_T_rc' & rob2_R) -> m_rob2_deadlock3') & 
        #((!m_rob2_deadlock1 & rob2_B_rc & rob2_B_rc' & rob2_B) -> m_rob2_deadlock1') & 
        #((!m_rob2_deadlock2 & rob2_B_rc & rob2_B_rc' & rob2_L) -> m_rob2_deadlock2') & 
        #((!m_rob2_deadlock3 & rob2_B_rc & rob2_B_rc' & rob2_D) -> m_rob2_deadlock3') & 
        #((!m_rob2_deadlock1 & rob2_L_rc & rob2_L_rc' & rob2_L) -> m_rob2_deadlock1') & 
        #((!m_rob2_deadlock2 & rob2_L_rc & rob2_L_rc' & rob2_B) -> m_rob2_deadlock2') & 
        #((!m_rob2_deadlock3 & rob2_L_rc & rob2_L_rc' & rob2_T) -> m_rob2_deadlock3') & 
        #((!m_rob2_deadlock1 & rob2_R_rc & rob2_R_rc' & rob2_R) -> m_rob2_deadlock1') & 
        #((!m_rob2_deadlock2 & rob2_R_rc & rob2_R_rc' & rob2_D) -> m_rob2_deadlock2') & 
        #((!m_rob2_deadlock3 & rob2_R_rc & rob2_R_rc' & rob2_T) -> m_rob2_deadlock3') & 
        #((!m_rob2_deadlock1 & rob2_D_rc & rob2_D_rc' & rob2_D) -> m_rob2_deadlock1') & 
        #((!m_rob2_deadlock2 & rob2_D_rc & rob2_D_rc' & rob2_B) -> m_rob2_deadlock2') & 
        #((!m_rob2_deadlock3 & rob2_D_rc & rob2_D_rc' & rob2_R) -> m_rob2_deadlock3')) )

    addedSystemSafetyStatementCoordination = "| ! & ! rob"+str(robNumA)+str(robNumB)+"_deadlock rob"+str(robNumA)+str(robNumB)+"_deadlock' "

    # Start by populating the '&'
    addedSystemSafetyRobotA = ""
    addedSystemSafetyRobotB = ""
    count = 0
    for region in regions:
        if region['name'] == 'boundary':
            pass
        else:

            regA = region['name']

            for face in faces:
                if (face[0] == regA and face[1] != 'boundary') or (face[1] == regA and face[0] != 'boundary'):
                    if count != 0:
                        addedSystemSafetyRobotA += "& "
                        addedSystemSafetyRobotB += "& "
                    count += 1
            if count != 0:
                addedSystemSafetyRobotA += "& "
                addedSystemSafetyRobotB += "& "

    for region in regions:
        if region['name'] == 'boundary':
            pass
        else:

            addedSystemSafetyRobotA += "| ! & & & ! m_rob"+str(robNumA)+"_deadlock1 rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' rob"+str(robNumA)+"_"+regA+" m_rob"+str(robNumA)+"_deadlock1' "
            addedSystemSafetyRobotB += "| ! & & & ! m_rob"+str(robNumB)+"_deadlock1 rob"+str(robNumB)+"_"+regA+"_rc rob"+str(robNumB)+"_"+regA+"_rc' rob"+str(robNumB)+"_"+regA+" m_rob"+str(robNumB)+"_deadlock1' "
            
            count = 1
            for face in faces:
                if (face[0] == regA and face[1] != 'boundary') or (face[1] == regA and face[0] != 'boundary'):

                    count += 1

                    if regA == face[0]:
                        regB = face[1]
                    else:
                        regB = face[0]

                    addedSystemSafetyRobotA += "| ! & & & ! m_rob"+str(robNumA)+"_deadlock"+str(count)+" rob"+str(robNumA)+"_"+regA+"_rc rob"+str(robNumA)+"_"+regA+"_rc' rob"+str(robNumA)+"_"+regB+" m_rob"+str(robNumA)+"_deadlock"+str(count)+"' "
                    addedSystemSafetyRobotB += "| ! & & & ! m_rob"+str(robNumB)+"_deadlock"+str(count)+" rob"+str(robNumB)+"_"+regA+"_rc rob"+str(robNumB)+"_"+regA+"_rc' rob"+str(robNumB)+"_"+regB+" m_rob"+str(robNumB)+"_deadlock"+str(count)+"' "

    addedSystemSafetyStatementCoordination += "| "+addedSystemSafetyRobotA+addedSystemSafetyRobotB

    return [addedSystemSafetyStatementCoordination]


def findAllEnvironmentPropositions(numRobots):
    """

    """

    addedEnvironmentPropositions = []
    addedEnvironmentInitialConditions = []

    # add singleton deadlock propositions
    for robNum in range(1, numRobots+1):

        addedEnvironmentPropositions.append("rob"+str(robNum)+"_deadlock")
        addedEnvironmentInitialConditions.append("! rob"+str(robNum)+"_deadlock")
        
    # add pairwise deadlock propositions
    for robNumA in range(1, numRobots+1):
        for robNumB in range(robNumA+1,numRobots+1):
                addedEnvironmentPropositions.append("rob"+str(robNumA)+str(robNumB)+"_deadlock")
                addedEnvironmentInitialConditions.append("! rob"+str(robNumA)+str(robNumB)+"_deadlock")

    return addedEnvironmentPropositions, addedEnvironmentInitialConditions


def findAllMempropMutexConditions(numRobots,numNeededDeadlockMempropsPerRobot):
    """

    """
    # # Restrict setting memprops only at the event of a deadlock
    # ( !m_rob1_deadlock1 & !(!rob1_deadlock & rob1_deadlock') & !(!rob2_deadlock & rob2_deadlock') & !(!rob12_deadlock & rob12_deadlock') ) -> (!m_rob1_deadlock1')
    # ( !m_rob1_deadlock2 & !(!rob1_deadlock & rob1_deadlock') & !(!rob2_deadlock & rob2_deadlock') & !(!rob12_deadlock & rob12_deadlock') ) -> (!m_rob1_deadlock2')
    # ( !m_rob1_deadlock3 & !(!rob1_deadlock & rob1_deadlock') & !(!rob2_deadlock & rob2_deadlock') & !(!rob12_deadlock & rob12_deadlock') ) -> (!m_rob1_deadlock3')
    # ( !m_rob2_deadlock1 & !(!rob1_deadlock & rob1_deadlock') & !(!rob2_deadlock & rob2_deadlock') & !(!rob12_deadlock & rob12_deadlock') ) -> (!m_rob2_deadlock1')
    # ( !m_rob2_deadlock2 & !(!rob1_deadlock & rob1_deadlock') & !(!rob2_deadlock & rob2_deadlock') & !(!rob12_deadlock & rob12_deadlock') ) -> (!m_rob2_deadlock2')
    # ( !m_rob2_deadlock3 & !(!rob1_deadlock & rob1_deadlock') & !(!rob2_deadlock & rob2_deadlock') & !(!rob12_deadlock & rob12_deadlock') ) -> (!m_rob2_deadlock3')

    addedSystemSafetyStatementMutex = []

    for i in range(1,numNeededDeadlockMempropsPerRobot+1):
        for robNum in range(1, numRobots+1):

            newStatement = "| ! & "
            
            for j in range(numRobots-1):
                newStatement += "& "

            newStatement += "! m_rob"+str(robNum)+"_deadlock"+str(i)+" ! & ! rob"+str(robNum)+"_deadlock rob"+str(robNum)+"_deadlock' "
            
            # pairwise deadlock propositions are ordered; go through each case, and add a condition if a match is found with the current robNum 
            # (will be numRob-1 of these)
            for robNumA in range(1, numRobots+1):
                for robNumB in range(robNumA+1,numRobots+1):
                    if robNumA == robNum or robNumB == robNum:
                        newStatement += "! & ! rob"+str(robNumA)+str(robNumB)+"_deadlock rob"+str(robNumA)+str(robNumB)+"_deadlock' "
            
            addedSystemSafetyStatementMutex.append(newStatement+"! m_rob"+str(robNum)+"_deadlock"+str(i)+"'")

    return addedSystemSafetyStatementMutex


def modifySpec(specFile, regionFile):
    regData = open(regionFile,"r")
    specData = open(specFile,"r")

    mode = ""
    lines = OrderedDict([("[INPUT]",[]),("[OUTPUT]",[]),("[ENV_TRANS]",[]),("[ENV_INIT]",[]),("[SYS_TRANS]",[]),("[SYS_INIT]",[]),("[ENV_LIVENESS]",[]),("[SYS_LIVENESS]",[]),("[OBSERVABLE_INPUT]",[]),("[UNOBSERVABLE_INPUT]",[]),("[CONTROLLABLE_INPUT]",[])])
    # data = []
    # with codecs.open(regionFile,'rU','utf-8') as regfile:
    #     for line in regfile:
    #         data.append(json.loads(line))

    faces, regions = parseRegionData(regData)

    regData.close()  

    for line in specData.readlines():
        line = line.strip()
        if line == "":
            pass
        elif line.startswith("["):
            mode = line
            # if not mode in lines:
            #    lines[mode] = []
        else:
            if mode=="" and line.startswith("#"):
                # Initial comments
                pass
            else:
                lines[mode].append(line)

    # print lines["[SYS_TRANS]"]
    for robNumA in range(1,NUM_ROBOTS+1):
        
        disabledTransitionConstraintStatements, addedSystemPropositions, addedSystemInitialConditions, numNeededDeadlockMempropsPerRobot = findDisabledTransitionConditions(regions,faces,robNumA)
        
        lines["[OUTPUT]"].extend(addedSystemPropositions)
        lines["[SYS_INIT]"].extend(addedSystemInitialConditions)
        lines["[SYS_TRANS]"].extend(disabledTransitionConstraintStatements)
        
        for robNumB in range(robNumA+1,NUM_ROBOTS+1):

            pairwiseDeadlockConstraintStatements = findPairwiseDeadlockConditions(regions,faces,robNumA,robNumB)
            pairwiseCoordinationConstraintStatements = findPairwiseCoordinationConditions(regions,faces,robNumA,robNumB)
            
            lines["[ENV_TRANS]"].extend(pairwiseDeadlockConstraintStatements)
            lines["[SYS_TRANS]"].extend(pairwiseCoordinationConstraintStatements)

    addedEnvironmentPropositions, addedEnvironmentInitialConditions = findAllEnvironmentPropositions(NUM_ROBOTS)
    mempropMutexConstraintStatements = findAllMempropMutexConditions(NUM_ROBOTS,numNeededDeadlockMempropsPerRobot)

    lines["[INPUT]"].extend(addedEnvironmentPropositions)
    lines["[ENV_INIT]"].extend(addedEnvironmentInitialConditions)
    lines["[SYS_TRANS]"].extend(mempropMutexConstraintStatements)

    #print lines["[SYS_TRANS]"]
    # print lines

    specData.close()

    with open("DR_"+specFile, 'w') as file:
        for mode in lines:
            # print lines[mode]
            if len(lines[mode]) > 0:
                file.writelines("\n"+mode+"\n")
                for i in range(len(lines[mode])):
                    file.writelines(lines[mode][i]+"\n")


if __name__ == "__main__":
    if len(sys.argv)<2:
        print >>sys.stderr, "Error: Need input file parameter"
        sys.exit(1)

    regionFile = sys.argv[1]
    specFile = sys.argv[2]

    modifySpec(specFile,regionFile)
