#!/usr/bin/env python
import serial
import time
import rospy
from geometry_msgs.msg import Point
from numpy import binary_repr
import numpy as np

class servo_controller:
    def __init__(self):
        self.sub = rospy.Subscriber("movement_data",Point,self.changeAngleCallback,queue_size=1)
        self.ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)  # open serial port
        print(self.ser.name)         # check which port was really used
        self.tiltAngle = 1250
        self.panAngle = 1500
        # self.basetime = rospy.Time.now()
        self.setAngle(0,self.tiltAngle)
        self.setAngle(1,self.panAngle)
        time.sleep(0.1)

    def target_low_bits(self,x):
        number_string = str(binary_repr(x*4, width=14))
        number_string = number_string[-7:]
        number_string = '0' + number_string
        return int(number_string, 2)

    def target_high_bits(self,x):
        number_string = str(binary_repr(x*4, width=14))
        number_string = number_string[:7]
        number_string = '0' + number_string
        return int(number_string, 2)

    def changeAngleCallback(self,movementData):
        deltaAnglePan = 0
        deltaAngleTilt = 0
        if movementData.x != 0:
            deltaAnglePan = int(movementData.x)/10
            if abs(deltaAnglePan) < 5 or abs(deltaAnglePan > 50):
                deltaAnglePan = 0
        if movementData.y != 0:
            deltaAngleTilt = int(movementData.y)/10
            if abs(deltaAngleTilt) < 5 or abs(deltaAngleTilt > 50):
                deltaAngleTilt = 0

        print "deltaAngleTilt =",deltaAngleTilt
        self.tiltAngle += deltaAngleTilt
        setTarget = chr(0x84)
        servoNumber = chr(0)
        posPart1 = chr(self.target_low_bits(self.tiltAngle))
        posPart2 = chr(self.target_high_bits(self.tiltAngle))
        commandStr = setTarget+servoNumber+posPart1+posPart2
        self.ser.write(commandStr)

        print "deltaAnglePan =",deltaAnglePan
        self.panAngle += deltaAnglePan
        setTarget = chr(0x84)
        servoNumber = chr(1)
        posPart1 = chr(self.target_low_bits(self.panAngle))
        posPart2 = chr(self.target_high_bits(self.panAngle))
        commandStr = setTarget+servoNumber+posPart1+posPart2
        self.ser.write(commandStr)

    def setAngle(self,servo,angle):
        setTarget = chr(0x84)
        servoNumber = chr(servo)
        posPart1 = chr(self.target_low_bits(angle))
        posPart2 = chr(self.target_high_bits(angle))
        commandStr = setTarget+servoNumber+posPart1+posPart2
        self.ser.write(commandStr)

def main():
    rospy.init_node('servo_controller', anonymous=True)
    sc = servo_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        ser.close()
        print("Shutting down")

if __name__ == '__main__':
    main()
