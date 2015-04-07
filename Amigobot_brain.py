import math
import libdw.sm as sm
from soar.io import io
import libdw.gfx as gfx
import libdw.util as util
import libdw.sonarDist as sonarDist
from libdw.boundarySM import *
# import libdw.eBotsonarDist as sonarDist
 
######################################################################
#
#            Brain SM
#
######################################################################
 
dDesired = 0.5
desiredRight = 0.35
forwardVelocity = 0.5
tolerance = 0.1
factor = math.cos(0.79)  # 45.3, 45 -> 0.785
upperBound = factor + tolerance  # close to wall, ratio higher
lowerBound = factor - tolerance  # further from wall, ratio lower
theta = math.pi / 4
# turningCounter = 30  # actual
# junctionForward = 45  # actual
turningCounter = 22  # for sim
junctionForward = 40  # for sim
maxReading = 2
rvelUpperLimit = 1
rvelLowerLimit = -1
 
# No additional delay
 
 
class Sensor(sm.SM):
 
    def getNextValues(self, state, inp):
        sensors = inp.sonars
        right = sonarDist.getDistanceRight(inp.sonars)
        if state is None:
            state = [sensors[5]] * 3
            median = [sensors[5]] * 3
        else:
            state.pop(0)
            state.append(sensors[5])
            median = state[:]
            median.sort()
        return state, (right, sensors, median[1])
 
# inp is the distance to the right
 
 
class WallFollower(sm.SM):
    startState = None
 
    def getNextValues(self, state, inp):
        # state = (movement, previousReadings, counter, timer)
        # state = ('F', (0.5, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], average), 0, 0)
        # print "input is,", inp
        if state is not None:
            currentRightFormula = inp[0]
            prevRightFormula = inp[1][0]
            prevRightFormulaAvg = inp[2]
            counter = state[2]
            sensors = inp.sensors
            timer = state[3]
            previousRightFront = state[1][1][3]
            previousRightDiag = state[1][1][4]
            previousRight = state[1][1][5]
            previousLeft = state[1][1][0]
            previousLeftDiag = state[1][1][1]
            previousLeftFront = state[1][1][2]
            previousFront = (previousRightFront + previousLeftFront) / 2.0
            print sensors
 
        if state is None:
            nextState = ('F', inp, 0, 0)
            return (nextState, io.Action(fvel=0.0, rvel=0.0))
 
        rvel = 0.0
        k3 = -2.5
        fvel = 0.2
 
        # Forward (F)
        if state[0] is 'F':
            k1 = 30
            k2 = -29.77
            e1 = desiredRight - currentRightFormula
            e2 = desiredRight - prevRightFormula
            # if not junctionDetected(previousLeftFront, previousLeftDiag,
            #                         previousRightFront, previousRightDiag,
            #                         previousLeft, previousRight):
 
            # Right sensor actually senses a wall, feel free to adjust
            if (prevRightFormulaAvg <= maxReading):
                rvel = (k1 * e1) + (k2 * e2)
                if rvel > rvelUpperLimit:
                    rvel = theta
                if rvel < rvelLowerLimit:
                    rvel = -theta
 
            if wallIsInfront(previousFront):
                fvel = 0.0
                nextState = ('W', inp, 0, 0)
                # probably an end point,
            else:
                nextState = ('F', inp, 0, 0)
 
            # junction, junction, ughh
            # front 4 are red or right 4 or left 4
            if (front4red(sensors) or right4red(sensors) or left4red(sensors)):
                # IT IS A JUNCTION OH NO
                # Check what is the next command
                print "Crap its a junction"
                rvel = 0.0
                fvel = 0.0
                nextState = ('J', inp, 0, 0)
 
        # Turning (T)
        if state[0] is 'T':
            obstacleSide = whereIsObstacle(
                previousRightFront, previousLeftFront)
            # turn while counter < 30
            if obstacleSide == False:  # turn right
                # counter += 1
                fvel = 0.0
                # rvel = -theta
                nextState = ('TR', inp, counter, 0)
            else:  # turn left by default
                # counter += 1
                fvel = 0.0
                # rvel = theta
                nextState = ('TL', inp, counter, 0)
                print "counter:", counter, "\n\n\n\n"
            # note: if counter is 30 or greater go to 'F' state, reset counter
            # if counter >= turningCounter:
            #     nextState = ('F', inp, 0, 0)
 
        # Turn Left (TL)
        if state[0] is 'TL':
            counter += 1
            fvel = 0.0
            rvel = theta
            nextState = ('TL', inp, counter, 0)
            print "turning left:", counter
            if counter >= turningCounter:
                print "done turning left"
                nextState = ('F', inp, 0, 0)
 
        if state[0] is 'TR':
            counter += 1
            fvel = 0.0
            rvel = -theta
            nextState = ('TR', inp, counter, 0)
            print "turning right:", counter
            if counter >= turningCounter:
                print "done turning right"
                nextState = ('F', inp, 0, 0)
 
        # Junction (J)
        if state[0] is 'J':
            print "You have entered the junction state! Hooray!"
            # counter += 1
            # rvel = 0.2
            # fvel = 0.3
            print "Move forward until your sides stick out"
            while stickout(sensors) != True:
                io.Action(fvel = 0.2, rvel = 0)
            # Imsert Code to turn here.

 
            if not (previousRight > maxReading or previousLeft > maxReading):
                fvel = 0.2
                nextState = ('J', inp, 0, 0)
            else:
                fvel = 0.2
                counter += 1
                nextState = ('J', inp, counter, 0)
                if counter >= junctionForward:
                 # check next command. go which way
                    nextState = ('H', inp, 0, 0)
 
        # Wait (W) - waiting. every tick is 0.1s. So 150 ticks is 15s.
        if state[0] is 'W':
            if timer <= 150:
                timer += 1
                fvel = 0.0
                rvel = 0.0
                nextState = ('W', inp, 0, timer)
            else:
                print 'DONE'
                nextState = ('H', inp, 0, 0)
 
        if state[0] is 'H':
            print "Halted!"
            fvel = 0.0
            rvel = 0.0
            nextState = ('H', inp, 0, 0)
 
        print "------------------------"
        print "State:", nextState[0]
        print "Counter:", nextState[2]
        print "Timer:", nextState[3]
        print "------------------------"
        return (nextState, io.Action(fvel=fvel, rvel=rvel))
 
 
sensorMachine = Sensor()
sensorMachine.name = 'sensor'
mySM = sm.Cascade(sensorMachine, WallFollower())
 
######################################################################
#
#            Running the robot
#
######################################################################
 
 
def wallIsInfront(value):
    if value < dDesired + tolerance:
        return True
    else:
        return False
 

def stickout(sensors):
    if sensors[0] > maxReading or sensors[5] > maxReading:
        return True
    return False

 
def front4red(sensors):
    l = []
    for i in sensors[1:5]:
        if i >= maxReading:
            l.append(True)
    if len(l) == 4:
        return True
    return False
 
 
def left4red(sensors):
    l = []
    for i in sensors[0:4]:
        if i >= maxReading:
            l.append(True)
    if len(l) == 4:
        return True
    return False
 
 
def right4red(sensors):
    l = []
    for i in sensors[2:6]:
        if i >= maxReading:
            l.append(True)
    if len(l) == 4:
        return True
    return False
 
 
def junctionDetected(leftFront, leftDiag, rightFront, rightDiag, left, right):
    if (leftFront >= maxReading) and \
            (leftDiag >= maxReading) and \
            (rightFront >= maxReading) and \
            (rightDiag >= maxReading) or \
            (left >= maxReading or right >= maxReading):
        return True
    return False
 
 
def whereIsObstacle(right, left):
    print right, left
    if right > left + tolerance:
        return 'left'
    elif left > right + tolerance:
        return 'right'
    else:
        # print 'readings about even, so just right turn'
        return 'right'
 
# def inRightRange(value):
#     if 0.3 <= value <= 0.5:
#         return True
#     else:
#         return False
 
 
# def inHypRatioRRange(value):
#     if (factor - tolerance) <= value <= (factor + tolerance):
#         return True
#     return False
 
 
# def tiltedIn(value):
#     if value > factor + tolerance:
#         return True
#     return False
 
 
# def tiltedAway(value):
#     if value < value - tolerance:
#         return True
#     return False
 
 
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
