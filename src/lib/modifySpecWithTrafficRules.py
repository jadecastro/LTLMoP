#!/usr/bin/python

import numpy as np
import sys
import StringIO
import re

robotRadius = 0.9
scalingPixelsToMeters = 70.

def polyArea(pts):
    """
    Find the area of a polygon, assuming vertices sorted in either CW or CCW order
    source: http://stackoverflow.com/questions/19873596/convex-hull-area-in-python
    """
    lines = np.hstack([pts,np.roll(pts,-1,axis=0)])
    area = 0.5*abs(sum(x1*y2-x2*y1 for x1,y1,x2,y2 in lines))
    return area

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
            print thisType

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
            print thisPoint
            thisSize = zip(*[iter(thisSize)]*2)
            print thisSize
            # thisPosition = filter(None, thisPosition) 
            thisPosition = zip(*[iter(thisPosition)]*2)
            print thisPosition
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

def findDoorConstraintConditions(faces):
    """
    Detect if a given region-region face resticts the number of robots that can simultaneously pass and generate the necessary LTL formulas

    NB: assume at most two robots
    """
    fullyObstructedFaces = []
    partiallyObstructedFaces = []
    addedSystemSafetyStatement = []
    for face in faces:
        if face[0] == 'boundary' or face[1] == 'boundary':
            pass
        else:
            x = np.array(face[2:])
            facesCoord = x.astype(np.float)
            faceWidth = 0
            for j in range(0,len(facesCoord)-2,2):
                # assume all faces are adjacent
                faceWidth += np.linalg.norm(facesCoord[j:j+2] - facesCoord[j+2:j+4])/scalingPixelsToMeters
            print faceWidth

            if faceWidth <= 2*robotRadius:
                print "yes"
                fullyObstructedFaces.append([face[:2]])
                A = face[0]
                B = face[1]
                newStatement = "& & & ! & rob1_"+A+"_rc' rob1_"+B+"' ! & rob1_"+B+"_rc' rob1_"+A+"' ! & rob2_"+A+"_rc' rob2_"+B+"' ! & rob2_"+B+"_rc' rob2_"+A+"'"
                print newStatement
                addedSystemSafetyStatement.append(newStatement)
            elif faceWidth <= 4*robotRadius:
                print "yes"
                partiallyObstructedFaces.append([face[:2]])
                A = face[0]
                B = face[1]
                # add a condition that prevents any two robots from passing through the door
                newStatement = "| ! | & rob1_"+A+"_rc' rob1_"+B+"' & rob1_"+B+"_rc' rob1_"+A+"' ! | & rob2_"+A+"_rc' rob2_"+B+"' & rob2_"+B+"_rc' rob2_"+A+"'"
                print newStatement
                addedSystemSafetyStatement.append(newStatement)
            else:
                print "no"

    return addedSystemSafetyStatement, fullyObstructedFaces, partiallyObstructedFaces

def findRegionCapacityConstraintConditions(regions):
    """
    Determine the maximum number of robots that can fit in a region and generate the necessary LTL formulas

    NB: assume at most two robots
    """
    addedSystemSafetyStatement = []
    for region in regions:
        A = 0
        if region['name'] == 'boundary':
            pass
        else:
            pts = []
            if region['type'] == 'poly':
                for coord in region['points']:
                    pts.append([float(coord[0])/scalingPixelsToMeters,float(coord[1])/scalingPixelsToMeters])
                A = polyArea(pts)
                print A
            elif region['type'] == 'rect':
                A = float(region['size'][0][0])/scalingPixelsToMeters * float(region['size'][0][1])/scalingPixelsToMeters
                print A
            if (A/(np.pi*robotRadius**2)) > 2:
                # the region can contain more than two robots
                pass
            elif (A/(np.pi*robotRadius**2)) > 1:
                # the region can contain at most two robots
                pass
            else:
                # the region can contain at most one robot
                # TODO: add the case of dynamic obstacles
                newStatement = "& | ! rob1_"+region['name']+"_rc' ! rob2_"+region['name']+"' | ! rob2_"+region['name']+"_rc' ! rob1_"+region['name']+"'"
                print newStatement
                addedSystemSafetyStatement.append(newStatement)

    return addedSystemSafetyStatement

def findRegionWidthConstraintConditions(regions,faces):
    """
    Determine if region width limits the number of robots that can pass in opposing directions and generate the necessary LTL formulas

    NB: assume at most two robots
    """

    addedOutputPropositions = []
    addedSystemSafetyStatement = []
    addedSystemInitialConditions = []
    memProps = [[],[]]
    for region in regions:
        A = 0
        if region['name'] == 'boundary':
            pass
        elif region['type'] == 'rect':
            A = float(region['size'][0][0])/scalingPixelsToMeters * float(region['size'][0][1])/scalingPixelsToMeters
            print A
            # limitingWidth = [region['size'][0][i]/scalingPixelsToMeters for i in range(0,2) if region['size'][0][i]/scalingPixelsToMeters < 4*robotRadius]
            if (A/(np.pi*robotRadius**2)) > 2:
                # the region can contain more than two robots
                for face in faces:
                    if (face[0] == region['name'] and face[1] != 'boundary') or (face[1] == region['name'] and face[0] != 'boundary'):
                        x = np.array(face[2:])
                        facesCoord = x.astype(np.float)
                        faceWidth = 0
                        for j in range(0,len(facesCoord)-2,2):
                            # assume all faces are adjacent
                            faceWidth += np.linalg.norm(facesCoord[j:j+2] - facesCoord[j+2:j+4])/scalingPixelsToMeters
                        print 'face width '+str(faceWidth)

                        if all(faceWidth != [float(region['size'][0][i])/scalingPixelsToMeters for i in range(0,2)]):
                            raise 'ERROR: each of the faces must match a side of the rectangle'

                        if faceWidth < 4*robotRadius:
                            if region['name'] == face[0]:
                                regA = face[0]
                                regB = face[1]
                            else:
                                regA = face[1]
                                regB = face[0]

                            #memProps[0].append("m_rob1_"+regB)
                            #memProps[1].append("m_rob2_"+regB)

                            # Add a platooning constraint
                            # newOutputPropositions = "m_rob1_"+regB+"\nm_rob2_"+regB
                            # addedOutputPropositions.append(newOutputPropositions)
                            # print newOutputPropositions
                            # newSysInitialConds = "& ! m_rob1_"+regB+" ! m_rob2_"+regB
                            # addedSystemInitialConditions.append(newSysInitialConds)
                            # print newSysInitialConds
                            # newStatement = "& | ! & & m_rob1_"+regB+"' rob1_"+regA+"_rc' ! rob2_"+regB+"_rc' ! rob2_"+regA+"' | ! & & m_rob2_"+regB+"' rob2_"+regA+"_rc' ! rob1_"+regB+"_rc' ! rob1_"+regA+"'"
                            # addedSystemSafetyStatement.append(newStatement)
                            # print newStatement
                            # newConditionsOnSettingMempropsRobot1 = "& & | ! rob1_"+regB+"_rc' m_rob1_"+regB+"' | ! & rob1_"+regA+"_rc' m_rob1_"+regB+" m_rob1_"+regB+"' | | rob1_"+regB+"_rc' rob1_"+regA+"_rc' ! m_rob1_"+regB+"'"
                            # addedSystemSafetyStatement.append(newConditionsOnSettingMempropsRobot1)
                            # print newConditionsOnSettingMempropsRobot1
                            # newConditionsOnSettingMempropsRobot2 = "& & | ! rob2_"+regB+"_rc' m_rob2_"+regB+"' | ! & rob2_"+regA+"_rc' m_rob2_"+regB+" m_rob2_"+regB+"' | | rob2_"+regB+"_rc' rob2_"+regA+"_rc' ! m_rob2_"+regB+"'"
                            # addedSystemSafetyStatement.append(newConditionsOnSettingMempropsRobot2)
                            # print newConditionsOnSettingMempropsRobot2

                            # Platooning constraint without memprops
                            newStatement = "| ! & & & rob1_"+regA+"_rc rob1_"+regB+" rob1_"+regA+"_rc' rob2_"+regB+"_rc ! rob2_"+regA+"'"
                            addedSystemSafetyStatement.append(newStatement)
                            print newStatement                            
                            newStatement = "| ! & & & rob1_"+regB+"_rc rob1_"+regA+" rob1_"+regB+"_rc' rob2_"+regA+"_rc ! rob2_"+regB+"'"
                            addedSystemSafetyStatement.append(newStatement)
                            print newStatement                            
                            newStatement = "| ! & & & rob2_"+regA+"_rc rob2_"+regB+" rob2_"+regA+"_rc' rob1_"+regB+"_rc ! rob1_"+regA+"'"
                            addedSystemSafetyStatement.append(newStatement)
                            print newStatement                            
                            newStatement = "| ! & & & rob2_"+regB+"_rc rob2_"+regA+" rob2_"+regB+"_rc' rob1_"+regA+"_rc ! rob1_"+regB+"'"
                            addedSystemSafetyStatement.append(newStatement)
                            print newStatement                            

    if newOutputPropositions != []:
        # add mutex conditions on setting memprops
        newMutexCondition = ""
        for j in range(len(memProps)):
            for i in range(len(memProps[j]) - 2):
                newMutexCondition += "& "
            for memProp in memProps[j]:
                newMutexCondition += "| ! "+memProp
                for memPropNeg in memProps[j]:
                    if memPropNeg == memProp:
                        pass
                    else:
                        newMutexCondition += " ! "+memPropNeg
                newMutexCondition += "\n"
        print newMutexCondition
        addedSystemSafetyStatement.append(newMutexCondition)

    return addedOutputPropositions, addedSystemSafetyStatement, addedSystemInitialConditions


def modifySpec(specFile, regionFile):
    regData = open(regionFile,"r")
    specData = open(specFile,"r")

    mode = ""
    lines = {"[ENV_TRANS]":[],"[ENV_INIT]":[],"[INPUT]":[],"[OUTPUT]":[],"[SYS_TRANS]":[],"[SYS_INIT]":[],"[ENV_LIVENESS]":[],"[SYS_LIVENESS]":[],"[OBSERVABLE_INPUT]":[],"[UNOBSERVABLE_INPUT]":[],"[CONTROLLABLE_INPUT]":[] }

    # data = []
    # with codecs.open(regionFile,'rU','utf-8') as regfile:
    #     for line in regfile:
    #         data.append(json.loads(line))

    faces, regions = parseRegionData(regData)

    regData.close()  

    doorConstraintStatements, fullyObstructedFaces, partiallyObstructedFaces = findDoorConstraintConditions(faces)
    capacityConstraintStatements = findRegionCapacityConstraintConditions(regions)
    addedOutputPropositions, widthConstraintStatements, addedSystemInitialConditions = findRegionWidthConstraintConditions(regions, faces)

    # print fullyObstructedFaces
    # print partiallyObstructedFaces

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
    #lines["[SYS_TRANS]"].extend(doorConstraintStatements)   # COMMENTED because isn't realizable despite revisions (terminates with no cuts)??
    lines["[SYS_TRANS]"].extend(capacityConstraintStatements)
    lines["[OUTPUT]"].extend(addedOutputPropositions)
    lines["[SYS_TRANS]"].extend(widthConstraintStatements)
    lines["[SYS_INIT]"].extend(addedSystemInitialConditions)
    #print lines["[SYS_TRANS]"]

    # print lines

    specData.close()

    with open("test_"+specFile, 'w') as file:
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
