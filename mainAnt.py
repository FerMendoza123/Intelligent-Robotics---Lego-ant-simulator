#!/usr/bin/env pybricks-micropython
#from mainAnt import stop
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import inspect
import urandom
import math
#import os.fork

#Initialize sensors
rCSensor = ColorSensor(Port.S4)
lCSensor = ColorSensor(Port.S1)
gyroSensor = GyroSensor(Port.S2, Direction.CLOCKWISE)


#Initialize motors
rightMo = Motor(Port.C)
leftMo = Motor(Port.B)
markerMo = Motor(Port.D)

listDist = []
listTurn = []

bBorderLabel="black"
foodLabel ="red"
traceLabel="green"
anthillLabel="blue"

#Labels for side of the sensor
right=1
left=-1


#Constants for wheels in terms of degrees
speed=400 #Speed for each wheel in terms of degres/second
turnSpeed=100 #Turning speed
factor=1.0072
#Turning speed for each wheel
velLTurn=turnSpeed
velRTurn=velLTurn*factor
#Moving speed for each wheel
velL=speed
velR=factor*velL
#Constants for wheels in terms of distance
####Actuali this is the 
radBet = 47 #The distance between both wheels (axis lenght)
radWh = 28 #The radius of wheels
#speed for each wheel in terms of Distance/second
wheelSpeed=(speed*radWh*math.pi/180)/1000 #units are given in mm/s

#Timer for movement
movingTimer = StopWatch()
#Variables used to control movement
currMovementTime = 0
currTurningAngle = 0
#logical flags for processes
movingFlag = False
turningFlag = False
arrivedByLine = False
markerDown = False



def turn(angle):
    global turningFlag
    #If the angle is 0 it is not necesary to turn
    if(angle!=0):
        #Side it will turn
        turnSen = angle/abs(angle) 
        #Setting the velocities
        velLT = turnSen*velLTurn
        velRT = -turnSen*velRTurn
        #Turning
        leftMo.run(velLT)
        rightMo.run(velRT)
        a=0
        gyroSensor.reset_angle(0)
        #Loop used to control the turning
        while abs(gyroSensor.angle())<abs(angle):
            a=a+1
        stop()
    #Storing the turning angle
    listTurn.append(angle)
    
def move(dist):
    global currMovementTime
    global movingFlag
    #Calculating the time
    time = ((dist*10/wheelSpeed))
    #Global var used to control the movement time
    currMovementTime = time
    movingFlag=True
    movingTimer.reset()
    #Moving
    leftMo.run_time(velL, time, then=Stop.BRAKE, wait=False)
    rightMo.run_time(velR, time, then=Stop.BRAKE, wait=False)
    #Storing the distance
    listDist.append(dist)
    

#Function for stopping the motors
def stop():
    leftMo.hold()
    rightMo.hold()

#Function that senses the color, it returns a flag for the color detected
#And the side, if it has not detected any landmark color returns none
def sense():
    rSens = rCSensor.rgb()
    lSens = lCSensor.rgb()
    senSide = label = None

    if(rSens[0]<20 and rSens[1]<20 and rSens[2]<20):
        senSide=right
        label=bBorderLabel
    else:
        if(lSens[0]<20 and lSens[1]<20 and lSens[2]<20):
            senSide=left
            label=bBorderLabel
        else: #Check if its red color
            if(rSens[0]>rSens[1] and rSens[0]>rSens[2] and rSens[0]>50 and rSens[1]<20 and rSens[2]<20):
                senSide=right
                label=foodLabel
            else:
                if(lSens[0]>lSens[1] and lSens[0]>lSens[2] and lSens[0]>50 and lSens[1]<20 and lSens[2]<20):
                    senSide=left
                    label=foodLabel
                else:#Check if its blue color
                    if(rSens[2]>rSens[0] and rSens[2]>rSens[1] and rSens[2]>25 and rSens[0]<20 and rSens[1]<20):
                        senSide=right
                        label=anthillLabel
                    else:
                        if(lSens[2]>lSens[0] and lSens[2]>lSens[1] and lSens[2]>25 and lSens[0]<20 and lSens[1]<20):
                            senSide=left
                            label=anthillLabel

                        else:#Check if its green color
                            if(rSens[1]>rSens[0] and rSens[1]>rSens[2] and rSens[1]>35 and rSens[0]<20 and rSens[2]<31):
                                senSide=right
                                label=traceLabel
                            else:
                                if(lSens[1]>lSens[0] and lSens[1]>lSens[2] and lSens[1]>35 and lSens[0]<20 and lSens[2]<31):
                                    senSide=left
                                    label=traceLabel
                                else:
                                    label="noone"                       
    return label,senSide



def avoidBorder(side):
    #First of all wwe have to save the current distance (if the rovot is moving straight)
    interruptMoving()
    #if it was detected by right sensor then side is 1 and the turn must by counter clockwise
    deg = 0
    while deg==0:
        deg = urandom.randint(90,180)*(-1*side)
    turn(deg)
    #Distance will be chosen in cm
    dist=urandom.randint(10,30)
    move(dist)



            
#Returns the resulting angle and distances since the last reset of memory
def sum ():
    #We have to invert the sign of each angle in order to know the correct values
    #We need to obtain the sum of vector with all distances acording to their angles
    currAngle = 0
    sumXDis = 0
    sumYDis = 0
    for i in range(len(listDist)):
        #Sum of the angles
        currAngle = currAngle + (-listTurn[i])
        currDist = listDist[i] 
        #Obtainign cosdinates for each vector
        currXDis = currDist * math.cos(math.radians(currAngle))
        currYDis = currDist * math.sin(math.radians(currAngle))
        #Sum of the distances
        sumXDis = sumXDis + currXDis
        sumYDis = sumYDis + currYDis
    #We don´t need to turn more than 360°, this line aplies module
    currAngle=(currAngle/abs(currAngle))*(abs(currAngle)%360)
    #Get a positive equivalent
    if(currAngle<0):
        currAngle=360+currAngle
    return sumXDis, sumYDis,  currAngle
    
#Obtains the distance and direction needed to go back
def getBackParam():
    sumX, sumY, angle=sum()
    #Obtaining the direction from the origin to the robot
    finalAngle = (math.atan(sumY/sumX)*180)/math.pi
    if(sumX<0):
        finalAngle+=180
    #Obtaining the distance from the origin to the robot
    backDistance = math.sqrt(sumX*sumX + sumY*sumY)
    #Obtaining the angle due the current direction and position of the robot
    angle=(angle/angle)*(abs(angle)%360)
    backAngle = - (round(finalAngle+180 - angle))%360
    return backDistance, backAngle

#if the robot was moving then the last distance has not been completed
#And it is replaced
def replaceLastDistance(time):
    #Obtaining the distance traveled with the current time
    newDist = (time*wheelSpeed)/10
    listDist[-1] = newDist

#If a movement has not been completed and the robot needs to stop then it is interrupted
def interruptMoving():
    global movingFlag
    if(movingFlag):
        stop()
        replaceLastDistance(movingTimer.time())
        movingFlag=False


def downMarker():
    global markerDown
    markerMo.run_target(200, 90, then=Stop.BRAKE, wait=True)
    markerDown = True

def upMarker():
    global markerDown
    markerMo.run_target(200, -90, then=Stop.BRAKE, wait=True)
    markerDown=False


def followLine(side): 
    global arrivedByLine
    interruptMoving()
    
    varWhile=True
    #First of all we need to calculate the angle of the line found, 
    #We know tath the line comes from origin so, we just need the point we found
    #to calculate the tangent
    sumX, sumY, currRobAngle=sum()
    #currRobAngle=225
    lineAngle = (math.atan(sumY/sumX)*180)/math.pi
    
    if(sumX<0):
        lineAngle+=180  
    #lineAngle=360

    #If angle between robot angle and line angle is less than 90 
    #It is aligned in the correct direction of the line, otherwhise
    #We must turn to the other side of the line
    if(not arrivedByLine and abs(lineAngle-currRobAngle)>90):
        turn(90*side)
        side=-side
    
    #runFlag tells us if robot motors are runing 
    runFlag=False
    color=None
    currSide=side

    senSide=0
    #It will follow the line until if finds food or until it finds anthill
    if(arrivedByLine):
        goal = anthillLabel
    else:
        goal = foodLabel
    while(color!=goal):
        color,senSide = sense()
        if(color!=traceLabel and not runFlag):
            leftMo.run(velL)
            rightMo.run(velR)
            runFlag = True
        if(color==traceLabel):
            stop()
            turn(10*senSide)
            runFlag=False
        #If the robot choosed the wrong side we must correct it
        if(goal==anthillLabel and color==foodLabel):
            goal=foodLabel
        if(color==anthillLabel and goal==foodLabel):
            goal=anthillLabel
    #this Flag is true only when the robot finds food
    #is used to know if the robot must follow back
    if(goal==anthillLabel):
        arrivedByLine=False
    else:
        arrivedByLine=True


def comeBack():
    interruptMoving()
    #If the robot arrived by line we just have to follow it again
    if(arrivedByLine):
        color=None
        while(color!=traceLabel):
            color,side=sense()
            turn(1)
        downMarker()
        followLine(side)
    else:
    #If the robot arrived by searching then computes the back parameters
    #Next it turns and moves the specified distance
        dis, angle = getBackParam()
        turn(angle)
        downMarker()
        move(dis)
    
#Deletes the current memory
def resetList():
    global listDist
    global listTurn
    listDist = []
    listTurn = []

#Function for moving control
def keepSearching():
    global movingFlag
    global currMovementTime
    #If its moving straight we must look out
    if(movingFlag):
        if(movingTimer.time()>currMovementTime):
            currMovementTime=0
            movingFlag=False
    else:
    #If not we must continue searching
        deg=urandom.randint(-90,90)
        turn(deg)
        #Distance will be chosen in cm
        dist=urandom.randint(20,40)
        move(dist)

while True:
    color,side = sense()
    #If the robot finds a trace
    if(color==traceLabel):
        followLine(side)
        color,side = sense()
    #It will find food when it detects green
    if(color==foodLabel):
        comeBack()
    #If the robot finds a border
    if(color==bBorderLabel):
        avoidBorder(side)
    else:
        #If it has arrived to the anthill
        if(color==anthillLabel):
            if(markerDown):
                upMarker()
        #If it has not detected anything
        keepSearching()
