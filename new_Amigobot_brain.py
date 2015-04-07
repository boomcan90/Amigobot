import math
import libdw.sm as sm
from soar.io import io
import libdw.gfx as gfx
import libdw.util as util
import libdw.sonarDist as sonarDist
from libdw.boundarySM import *
import copy

## BRAIN SM

desiredFront = 0.7
desiredRight = 0.7
speedForward = 0.5
tolerance = 0.1
factor = 45
upperBound = factor + tolerance
lowerBound = factor - tolerance
turnAngle = math.pi/4
counterT = 30
juncFwd = 40
maxRangeOfSensor = 2.5
rvelLimitHigh = 1
rvelLimitLow = -1


class Sensor(sm.SM):

    def getNextValues(self, state, inp):
        sensorValueList = inp.sonars
        rightFunction = sonarDist.getDistanceRight(sensorValueList)
        if state is None:
            state = [sensorValueList[5], sensorValueList[5], sensorValueList[5]]
            median = state
        else:
            state.pop(0)
            state.append(sensorValueList[5])
            median = copy.deepcopy(state)  
            median.sort()
        rightMidValue = median[1]
        return state, (rightFunction, rightMidValue, sensorValueList)


class wallFollower(sm.SM):

    state = None


    def getNextValues(self, state, inp):
        if state is None:
            state = ("Fwd", 0, 0, inp)
            return (state, io.Action(fvel = 0.0, rvel = 0.0))
            # state = ( move, timer, counter, (rightFunction, rightMidValue, sensorValueList))
            #           ----------------------                                               -
            #                                                 THIS IS INP

        if not state is None:
            rightFunctionNow = inp[0]
            rightFunctionAgo = state[3][0]
            rightMidValueNow = inp[1]
            rightMidValueAgo = state[3][1]
            counter = state[2]
            timer = state[1]
            move = state[0]
            prevSensors = state[3][2]
            nowSensors = inp[2]

        fvel = 0.2
        rvel = 0.0
        c3 = -2.5    

        ## MOVING FORWARD

        if move is "Fwd":
            c1 = 30.00
            c2 = -29.77
            val1 = desiredRight - rightFunctionNow
            val2 = desiredRight - rightFunctionAgo

            if rightFunctionAgo <= maxRangeOfSensor:
                rvel = (c1 * val1) + (c2 * val2)
                # if rvel > rvelLimitHigh:
                #     rvel = turnAngle
                # if rvel < rvelLimitLow: 
                #     rvel = -turnAngle

            ## Checking if there is a wall in front:

            ## Checking previous set of sensors if there is a wall
            if prevSensors[2]>prevSensors[3]+2:
                prevFront = 0.9 * prevSensors[3]
            elif prevSensors[2]+2 < prevSensors[3]:
                prevFront = 0.9 * prevSensors[3]
            else:
                prevFront = (prevSensors[2] + prevSensors[3]) / 2.0


            ## Checking current set of data if there is a wall - using 2 set of data so that there is no change of state due to phantom values.
            if nowSensors[2]>nowSensors[3]+2:
                nowFront = 0.9 * prevSensors[3]
            elif nowSensors[2]+2 < nowSensors[3]:
                nowFront = 0.9 * nowSensors[3]
            else:
                nowFront = (nowSensors[2] + nowSensors[3]) / 2.0


            if frontWallPresent(prevFront) and frontWallPresent(nowFront):
                fvel = 0.0
                state = ("Wait", 0, 0, inp)
            elif (junctionCheck(nowSensors[0:4]) or junctionCheck(nowSensors[1:5]) or junctionCheck(nowSensors[2:6])): 
                fvel = 0.0
                rvel = 0.0
                state = ("Junc", 0, 0, inp)
            else:
                # print "Previous front is", prevFront, "\n"
                # print "Now front is ", nowFront, "\n"
                # print inp[2]
                state = ("Fwd", 0, 0, inp)




        ## Waiting for 5 seconds. 

        if move is "Junc":
            if not stickout(nowSensors):
                print "Junc"
                state = ("MoveAfterJunc", 0, 0, inp)
                return (state, io.Action(fvel = fvel, rvel = rvel))
            # state = ("End", 0, 0, inp)


        if move is "Wait":
            if timer <= 50:
                fvel = 0.0
                rvel = 0.0
                timer +=1
                state = ("Wait", timer, 0, inp)
            else:
                print "Stopping"
                state = ("MoveAfterJunc", timer, 0, inp)
        

        if move is "MoveAfterJunc":
            if timer <= 30:
                fvel = 0.2
                rvel = 0.0
                timer += 1
                state = ("MoveAfterJunc", timer, 0, inp)
            else: 
                print "End"
                state = ("End", 0, 0, inp)

        ## End of movement. 

        if move is "End":
            fvel = 0.0
            rvel = 0.0
            state = ("End", 0, 0, inp)




        return (state, io.Action(fvel = fvel, rvel = rvel))



sensorMachine = Sensor()
sensorMachine.name = 'sensor'
mySM = sm.Cascade(sensorMachine, wallFollower())


def frontWallPresent(frontDistance):
    if frontDistance < desiredFront + tolerance:
        return True 
    return False


def stickout(sensors):
    if sensors[0] > maxRangeOfSensor and sensors[5] > maxRangeOfSensor:
        return True
    print sensors[0], sensors[5]
    return False


def junctionCheck(sensors):
    k=0
    for i in sensors:
        if i > 4:
            k+=1
    if k == 4:
        return True
    return False


def setup():
    robot.gfx = gfx.RobotGraphics(drawSlimeTrail=False)
    # robot.gfx.addStaticPlotSMProbe(y=('rightDistance', 'sensor',
    #                                   'output', lambda x: x))
    robot.behavior = mySM
    robot.behavior.start(traceTasks=robot.gfx.tasks())
 
 
def step():
    robot.behavior.step(io.SensorInput()).execute()
    io.done(robot.behavior.isDone())
 
 
def brainStop():
    pass
