

import serial
import time

arduinoData = serial.Serial('com3', 115200)
time.sleep(1)


while True:
    while (arduinoData.inWaiting() == 0):
        pass

    dataPacket = arduinoData.readline()
    dataPacket = str(dataPacket, 'utf-8')
    splitPacket = dataPacket.split(',')
    
    
    theta = float(splitPacket[0]) #pitch 
    phi = float(splitPacket[1])  #roll
    yaw = float(splitPacket[2]) #yaw
    #calibration
    accelC = float(splitPacket[3])
    gyroC = float(splitPacket[4])
    mg = float(splitPacket[5])
    system = float(splitPacket[6])

    print("Theta: ", theta, "Phi: " , phi, "Yaw: ", yaw)
    print("AccelC: ", accelC, "GyroC: ", gyroC, "MgC: ", mg, "SysC: ", system)
   