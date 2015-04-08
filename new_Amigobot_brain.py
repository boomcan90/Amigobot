import math
import libdw.sm as sm
from soar.io import io
import libdw.gfx as gfx
import urllib2
import libdw.sonarDist as sonarDist
import libdw.search as Search

######################################################################
#
#            Brain SM
#
######################################################################

testing = True
 
# Forward threshold and side threshold
dDesired = 0.7
dRight = 0.7
tolerance = 0.1

turningAngle = math.pi / 4

maxSensorReading = 3
maxReadingOverrideForTurning = 5
rVelUpperLimit = 1
rVelLowerLimit = -1
hadWallBefore = False
timerW = 150


if testing:
    counterTurn = 21
    counterJunction = 35
    counterRotation = 42
    counterExitRotation = 15
    k1 = 30
    k2 = -29.77
else:
    counterTurn = 31
    counterJunction = 28
    counterRotation = 63
    counterExitRotation = 20
    k1 = 100
    k2 = -97.95

class Sensor(sm.SM):
 
    def getNextValues(self, state, inp):
        sensors = inp.sonars
        if state is None:
            state = [0, 0]
            state[0] = [[i, i, i] for i in sensors]
            state[1] = [sonarDist.getDistanceRight(sensors)] * 3
            medianSensorRight = state[1]
            medianSensors = sensors
        else:
            medianSensors = []
            for idx, sensor in enumerate(state[0]):
                # each sensor sort it
                state[0][idx].pop(0)
                state[0][idx].append(sensors[idx])
                temp = state[0][idx][:]
                temp.sort()
                medianSensors.append(temp[1])
            state[1].pop(0)
            state[1].append(sonarDist.getDistanceRight(sensors))
            medianSensorRight = state[1][:]
            medianSensorRight.sort()
        return state, (medianSensorRight[1], medianSensors)

 
class WallFollower(sm.SM):
    # start with None to get first sensor reading
    startState = None
 
    # should move forward to X first
    hadWallBefore = False
 
    # right and left depending on orientation and path list
    right = 2
    left = 0
 
    # generated path that the robot should follow
    path = []
    currentMoveCommand = ('Default', 'Default')
    nextMoveCommand = ('Default', 'Default')
 
    def getNextValues(self, state, inp):
        # state = (movement, previousReadings, counter, timer)
        # state = ("Fwd, (0.5, [0.5, 0.5, 0.5, 0.5, 0.5, 0.5], average), 0, 0)
        if state is not None:
            rightFormulaNow = inp[0]
            rightFormulaAgo = state[1][0]
            counter = state[2]
            sensors = inp[1]
            timer = state[3]
            rightFrontSensorAgo = state[1][1][3]
            rightDiagonalSensorAgo = state[1][1][4]
            rightSensorAgo = state[1][1][5]
            leftSensorAgo = state[1][1][0]
            leftFrontSensorAgo = state[1][1][2]
            move = state[0]
            if rightFrontSensorAgo > leftFrontSensorAgo + 2:
                previousFront = 0.9 * leftFrontSensorAgo
            elif leftFrontSensorAgo > rightFrontSensorAgo + 2:
                previousFront = 0.9 * rightFrontSensorAgo
            else:
                previousFront = (rightFrontSensorAgo + leftFrontSensorAgo) / 2.0

        if state is None:
            state = ('Fwd', inp, 0, 0)
            return (state, io.Action(fvel=0.0, rvel=0.0))
 
        rvel = 0.0
        k3 = -2.0
        fvel = 0.5
 
        # Forward (F)
        if move is 'Fwd':


            e1 = dRight - rightFormulaNow
            e2 = dRight - rightFormulaAgo
 
            if (rightSensorAgo <= maxSensorReading and
                    rightDiagonalSensorAgo <= maxSensorReading):
                rvel = (k1 * e1) + (k2 * e2)
 
            if wallIsInfront(previousFront):
                if self.hadWallBefore is False:
                    self.currentMoveCommand = self.path.pop(0)
                    self.hadWallBefore = True
                    if not (previousFront >= maxSensorReading):
                        fvel = (k3 * (dDesired - previousFront))
                    else:
                        fvel = 0.8
                state = ('Wait', inp, 0, 0)
            else:
                if not (previousFront >= maxSensorReading):
                    fvel = (k3 * (dDesired - previousFront))
                else:
                    fvel = 0.8
                state = ('Fwd', inp, 0, 0)
 
            print rightSensorAgo, leftSensorAgo


            if (rightSensorAgo >= maxReadingOverrideForTurning or leftSensorAgo >= maxReadingOverrideForTurning) \
                    and self.hadWallBefore:
                print "No Junc"
                rvel = 0.0
                fvel = 0.0
                counter += 1
                state = ('Fwd', inp, counter, 0)
                if counter >= 5:
                    state = ('Junc', inp, 0, 0)
 

        if move is 'turnLeft':
            counter += 1
            fvel = 0.0
            rvel = turningAngle
            state = ('turnLeft', inp, counter, 0)
            if counter >= counterTurn:
                state = ('exitJuncion', inp, 0, 0)
 
        if move is "turnRight":
            counter += 1
            fvel = 0.0
            rvel = -turningAngle
            state = ("turnRight", inp, counter, 0)
            if counter >= counterTurn:
                state = ("exitJunction", inp, 0, 0)
 
        # Junction (J)
        if move is "Junc":

            # uh why did i do this again?
            if not (rightSensorAgo > maxSensorReading or leftSensorAgo > maxSensorReading):
                fvel = 0.2
                state = ("Junc", inp, 0, 0)
            else:
                fvel = 0.2
                counter += 1
                state = ("Junc", inp, counter, 0)
                if counter >= counterJunction:
                    #  just to check if its a junction
                    if self.currentMoveCommand[1][0] == "Junc":
                        self.currentMoveCommand = self.nextMoveCommand[:]
                        if not len(self.path) > 0:
                            self.nextMoveCommand = (0, 0)
                        else:
                            self.nextMoveCommand = self.path.pop(0)
                        if self.currentMoveCommand[0] == self.right:
                            state = ("turnRight", inp, 0, 0)
                            self.right = (self.currentMoveCommand[0] + 1) % 4
                            self.left = (self.currentMoveCommand[0] - 1) % 4
                        elif self.currentMoveCommand[0] == self.left:
                            state = ("turnLeft", inp, 0, 0)
                            self.right = (self.currentMoveCommand[0] + 1) % 4
                            self.left = (self.currentMoveCommand[0] - 1) % 4
                        else:
                            state = ("exitJunction", inp, 0, 0)
                    else:
                        state = ("exitJunction", inp, 0, 0)
 
        if move is "exitJunction":
            counter += 1
            fvel = 0.2
            state = ("exitJunction", inp, counter, 0)
            if counter >= counterJunction + 20:
                state = ("Fwd", inp, 0, 0)
 
        if move is "Wait":
            if timer <= timerW:
                timer += 1
                fvel = 0.0
                rvel = 0.0
                state = ("Wait", inp, 0, timer)
            else:
                state = ("rotate", inp, 0, 0)
 
        # Rotate (R) - Rotate around.
        if move is "rotate":
            counter += 1
            fvel = 0.0
            rvel = turningAngle
            if counter >= counterRotation:
                if len(self.path) > 0:
                    # Check for next command, set current Command
                    print self.path[0]
                    self.currentMoveCommand = self.path.pop(0)  # (1, "Junc)
                    self.nextMoveCommand = self.path.pop(0)  # (2, 'J1')
                    self.right = (self.currentMoveCommand[0] + 1) % 4
                    self.left = (self.currentMoveCommand[0] - 1) % 4
                    print self.path[0]
                    state = ("exitRotation", inp, 0, 0)
                else:
                    # probably end of route
                    state = ("Halt", inp, 0, 0)
            else:
                state = ("rotate", inp, counter, 0)
 
        # Exit Rotation (ER) - Exit the rotation state
        if move is "exitRotation":
            e1 = dRight - rightFormulaNow
            e2 = dRight - rightFormulaAgo
            rvel = (k1 * e1) + (k2 * e2)
            counter += 1
            state = ("exitRotation", inp, counter, 0)
            if counter >= counterExitRotation:
                state = ("Fwd", inp, 0, 0)
 
        if move is "Halt":
            fvel = 0.0
            rvel = 0.0
            state = ("Halt", inp, 0, 0)

        print move
        print self.hadWallBefore
        print self.path[0]
        return (state, io.Action(fvel=fvel, rvel=rvel))
 
 
sensorMachine = Sensor()
sensorMachine.name = 'sensor'
wallFollower = WallFollower()
wallFollower.name = 'wallFollower'
# mySM = sm.Cascade(sensorMachine, WallFollower())
 
######################################################################
#
#            Running the robot & our custom functions
#
######################################################################
 
 
def wallIsInfront(value):
    if value < dDesired + tolerance:
        return True
    else:
        return False
 
 
def front4red(sensors):
    l = []
    for i in sensors[1:5]:
        if i >= maxSensorReading:
            l.append(True)
    if len(l) == 4:
        return True
    return False
 
 
def left4red(sensors):
    l = []
    for i in sensors[0:4]:
        if i >= maxSensorReading:
            l.append(True)
    if len(l) == 4:
        return True
    return False
 
 
def right4red(sensors):
    l = []
    for i in sensors[2:6]:
        if i >= maxSensorReading:
            l.append(True)
    if len(l) == 4:
        return True
    return False
 
 
def junctionDetected(leftFront, leftDiag, rightFront, rightDiag, left, right):
    if (leftFront >= maxSensorReading) and \
            (leftDiag >= maxSensorReading) and \
            (rightFront >= maxSensorReading) and \
            (rightDiag >= maxSensorReading) or \
            (left >= maxSensorReading or right >= maxSensorReading):
        return True
    return False
 
 
def whereIsObstacle(right, left):
    if right > left + tolerance:
        return 'left'
    elif left > right + tolerance:
        return 'right'
    else:
        return 'right'
 
 
# load data from a url
def loadData(testUrl):
    data = urllib2.urlopen(testUrl)
    data = data.readlines()
    data = [(k, int(v))
            for k, v in [line.strip().split() for line in data]]
    return data
 
 
# map nodes
# [left, up, right, down'] or [0,1,2,3]
# which ever you prefer
# (index-1)%4 -> gives the new left
# (index+1)%4 -> gives the new right
 
mapLv1 = {
    'A': [None, None, None, 'J1'],
    'B': [None, None, 'J1', None],
    'C': [None, 'J1', None, None],
    'X': ['J1', None, None, None],
    'J1': ['B', 'A', 'C', 'X']
}
 
mapLv2 = {
    'Z': [None, None, 'J1', None],
    'X': ['J2', None, None, None],
    'J1': ['Z', 'J4', 'J2', None],
    'J2': ['J1', 'J3', 'X', None],
    'J3': ['J4', 'B', 'A', 'J2'],
    'J4': ['D', 'C', 'J3', 'J1'],
    'A': ['J3', None, None, None],
    'B': [None, None, None, 'J3'],
    'C': [None, None, None, 'J4'],
    'D': [None, None, 'J4', None]
}

level = mapLv2

 
##########################################
# search SM
# Explanation to be added... hopefully...
##########################################
 
 
class genSearch(sm.SM):
 
    startState = None
    legalInputs = [0, 1, 2, 3]  # left, up, right, down
 
    def __init__(self, goal, whichMap):
        self.goal = goal
        self.selectedMap = whichMap
 
    def nextState(self, state, action):
        return self.selectedMap[state][action]
 
    def getNextValues(self, state, action):
        nextState = self.nextState(state, action)
        return nextState, nextState
 
    def done(self, state):
        return state == self.goal
 
testUrl = "http://people.sutd.edu.sg/~oka_kurniawan\
/10_009/y2015/2d/tests/level1_2.inp"
 
# prepare the path lah, if not what?
# max per trip is
 
 
def preparePath(mapToUse):
    data = loadData(testUrl)  # [('A', 10), ('B', 3), ('C', 15)]
    path = []
    pathTuple = [(a, ((b/6)+1)) for a, b in data]
    for idx, val in enumerate(pathTuple):
        # calculate forward path then calculate reverse path
        # val[0] is goal, val[1] is number of trips
        start = 'X'
        goal = val[0]
        forward = Search.smSearch(
            genSearch(goal, mapToUse), start, depthFirst=False, DP=True)
        reverse = Search.smSearch(
            genSearch(start, mapToUse), goal, depthFirst=False, DP=True)
        path.extend((forward + reverse) * val[1])
    return path
 

 
def setup():
    robot.gfx = gfx.RobotGraphics(drawSlimeTrail=False)
    # wallFollower.path = preparePath(level)
    wallFollower.path = [(None, 'X'), (0, 'J2'), (0, 'J1'), (1, 'J4'), (1, 'C'), (None, 'C'), (3, 'J4'), (2, 'J3'), (3, 'J2'), (2, 'X')]
    mySM = sm.Cascade(sensorMachine, wallFollower)
    robot.behavior = mySM
    robot.behavior.start(traceTasks=robot.gfx.tasks())
 
 
def step():
    robot.behavior.step(io.SensorInput()).execute()
    io.done(robot.behavior.isDone())
 
 
def brainStop():
    pass
