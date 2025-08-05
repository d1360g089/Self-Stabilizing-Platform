import vpython
from vpython import *
import numpy as np
import serial
import time

arduinoData = serial.Serial('com3', 115200)
time.sleep(1)

scene.range = 5
scene.width = 800
scene.height = 600

breadboardL = 7

bnolength = 1
nanolength = 2



breadboard = box(length=breadboardL, width = 2, height = .2, color=color.white, opacity=.5)




bno = box(pos=vector(-.5,.2,0), length = bnolength, width=.75, height=.2, color=color.blue, opacity=.7)
nano = box(pos=vector(-2.4,.2,0),length = nanolength, width=.5, height=.15, color=color.green, opacity=.7)


xVector = arrow(axis=vector(0,0,1), color=color.red, length=3, shaftwidth=.05, height=.1)
yVector = arrow(axis=vector(0,1,0), color=color.green, length=3, shaftwidth=.05, height=.1)
zVector = arrow(axis=vector(-1,0,0), color=color.blue, length=3, shaftwidth=.05, height=.1)

frontArrowL = 2
frontArrow = arrow(axis=vector(0,0,1), color=color.purple, length=frontArrowL, shaftwidth=.095, height=.1)

upArrowL = 2
upArrow = arrow(axis=vector(0,1,0), length=1, shaftwidth=.1, color=color.magenta, height=.1)

sideArrowL = 2
sideArrow = arrow(axis=vector(0,0,1), length=sideArrowL, shaftwidth=.1, height=.1, color=color.orange)



myObj = compound([breadboard, bno, nano])

while True:


    while arduinoData.inWaiting() == 0:
        pass

    dataPacket = arduinoData.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitpacket = dataPacket.split(",")

    qW = float(splitpacket[0])
    qX = float(splitpacket[1])
    qY = float(splitpacket[2]) 
    qZ = float(splitpacket[3])
    #calibration
    #accelC = float(splitpacket[3])
    #gyroC = float(splitpacket[4])
    #mg = float(splitpacket[5])
    #system = float(splitpacket[6])

 
    roll = -np.atan2(2*(qW*qX + qY*qZ), 1-2*(qX*qX + qY*qY))
    theta = np.asin(2*(qW*qY - qX*qZ))
    yaw = -np.atan2(2*(qW*qZ + qX*qY), 1-2*(qY*qY + qZ*qZ))
    

    rate(100)
    
    xVal = breadboardL*np.cos(yaw)*np.cos(theta)
    yVal = breadboardL*np.sin(theta)
    zVal = breadboardL*np.sin(yaw)*np.cos(theta)

    k = vector(xVal,yVal,zVal)
        
    y = vector(0,1,0)
    s = cross(k,y)
    v = cross(s,k)
    v_rot = v*np.cos(roll) + cross(k, v)*np.sin(roll) 

    frontArrow.axis = k #(1,0,0)
    sideArrow.axis = cross(k, v_rot) #vector(0,0,1)
    upArrow.axis = v_rot # vector(0,1,0)

    frontArrow.length = 4
    sideArrow.length = 2
    upArrow.length = 2
    myObj.axis = k
    myObj.up = v_rot

    
    
    























      




