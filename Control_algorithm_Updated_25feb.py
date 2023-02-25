#!/usr/bin/env python
PKG = 'control'
from cmath import sqrt
#from this import s
import roslib; roslib.load_manifest(PKG)
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Twist
import subprocess
import math
import numpy as np
import time
import serial

 
 class state_machine(object):
    def __init__(self):
        self.arduino = serial.Serial(port='/dev/ttyUSB0' , baudrate = 9600, timeout = 1) #Communication with Arduino


        #added
        self.right_thruster = "right_thruster"
        self.left_thruster = "left_thruster"
        self.bow_thruster = "bow_thruster"
        self.right_servo = "right_servo"
        self.left_servo = "left_servo"
        self.x = 0
        self.y = 0
        self.z = 0
        self.angle = 0


        while not rospy.is_shutdown():
            rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)

   
    def cmd_callback(self, cmd_data):
        self.x = float(cmd_data.linear.x) #Do we need "float"?
        self.y = float(cmd_data.linear.y)
        self.angle = float(cmd_data.angular.z)
        self.dist = sqrt((self.x)^2 + (self.y)^2)


    def move(self, motor, angle):
        i=0
        if motor == "right_thruster":
            i = 1
        elif motor == "left_thruster":
            i = 2
        elif motor == "bow_thruster":
            i = 3
        elif motor == "right_servo":
            i = 4
        elif motor == "left_servo":
            i = 5
        message = str(angle + 1000 * i )  #the i*1000 is for decoding the motor we want to move via the send message.
        print("sending move message: ")
        print(message)
        self.arduino.write(bytes(message).encode("utf-8"))
        time.sleep(0.01)  # in order to make sure the arduino will read different messages as different messages




    def Situation(self):
        # Low values
        if abs(self.dist) <= 0.1:
            self.move(self.right_thruster, 90)  # Stop propeller=90
            self.move(self.left_thruster, 90)

        # "weak" (less than 3)
        if 0.1<self.dist<=3:
            if -90 <= self.angle <= 90: # Forward
                self.move(self.right_servo, (90 + self.angle))  # Setting value to be between 90 to 180
                self.move(self.left_servo, (90 + self.angle))
                self.move(self.right_thruster, (90+self.dist*30)) #Setting value to be between 90 to 180
                self.move(self.left_thruster, (90+self.dist*30))
            if abs(self.angle) > 90: # Backward
                if 0 < self.angle:
                    self.move(self.right_servo, (180-self.angle))
                    self.move(self.left_servo, (180-self.angle))
                    self.move(self.right_thruster, 130)
                    self.move(self.left_thruster, 130)
                if self.angle < 0:
                    self.move(self.right_servo, (-180+self.angle))
                    self.move(self.left_servo, (-180+self.angle))
                    self.move(self.right_thruster, 130)
                    self.move(self.left_thruster, 130)

        # "strong" (more than 3)
        if self.dist > 3:
            if -90 <= self.angle <= 90:  # Forward
                self.move(self.right_servo, (90 + self.angle))  # Setting value to be between 90 to 180
                self.move(self.left_servo, (90 + self.angle))
                self.move(self.right_thruster, 180) #Maximum power forward=180
                self.move(self.left_thruster, 180)
            if abs(self.angle) > 90: # Backward
                if 0 < self.angle:
                    self.move(self.right_servo, (180))
                    self.move(self.left_servo, (180))
                    self.move(self.right_thruster, 180)  # Maximum power forward=180
                    self.move(self.left_thruster, 180)
                if self.angle < 0:
                    self.move(self.right_servo, (0))
                    self.move(self.left_servo, (0))
                    self.move(self.right_thruster, 180)  # Maximum power forward=180
                    self.move(self.left_thruster, 180)



            if __name__ == '__main__':
  rospy.init_node('listener', anonymous=True)
  b = state_machine()
