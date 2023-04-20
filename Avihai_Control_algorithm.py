#!/usr/bin/env python
PKG = 'control'
from cmath import sqrt
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
        self.angle = 0
        self.dist=0

        while not rospy.is_shutdown():
            rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
            try :
                self.Situation()
            except KeyboardInterrupt :
                self.move(self.right_thruster, 90)  # Stop propeller=90
                self.move(self.left_thruster, 90)
                self.move(self.bow_thruster, 90) 
            
    def cmd_callback(self, cmd_data):
        self.x = cmd_data.linear.x
        if self.x == None or abs(self.x) < 0.1 :
            self.x = 0
        self.y = cmd_data.linear.y
        if self.y == None :
            self.y = 0
        self.angle = math.degrees(cmd_data.angular.z)
        if self.angle == None :
            self.angle = 0
        self.dist = math.sqrt((self.x)**2 + (self.y)**2)
        if self.angle == 0 :
            try :
                if self.y !=0 :
                    self.angle = math.degrees(math.atan2(self.y,self.x))
                else:
                    self.angle = 0
            except ZeroDivisionError :
                if self.y > 0 :
                    self.angle = 90
                elif self.y < 0 :
                    self.angle = -90

    def move(self, motor, angle):
        i=0
        if motor == "right_thruster":
            i = 1
        elif motor == "left_thruster":
            i = 2
        elif motor == "bow_thruster":
            i = 3
        message = str(angle + 1000 * i )
        print("motor : "+str(motor)+ " ; angle value "+str(angle) +" ! ")
        self.arduino.write(bytes(message).encode("utf-8"))

        time.sleep(0.05)  # in order to make sure the arduino will read different messages as different messages

    def Situation(self):
         proportional_angle = 90+(self.angle/2)
         proportional_thrust = 90+(self.dist*15)/2
         while True:
                # reverse in close position to the target
                if -1 < self.x < -0.1 and abs(self.y) <= 1:
                    while -1 < self.x < -0.1 :
                        self.move(self.right_thruster, 70)
                        self.move(self.left_thruster, 70)
                        self.move(self.bow_thruster, 90)
                    
                # Low values
                elif abs(self.dist) <= 0.1 and abs(self.angle) <= 3:
                    self.move(self.right_thruster, 90)
                    self.move(self.left_thruster, 90)
                    self.move(self.bow_thruster, 90) 
                    time.sleep(0.1)

                elif abs(self.dist) <= 0.1 and abs(self.angle) > 3:
                    self.move(self.right_thruster, 90)
                    self.move(self.left_thruster, 90)
                    self.move(self.bow_thruster, proportional_angle)
                    time.sleep(0.1)


                # "weak" (less than 6)
                elif 0.1<self.dist<=6:
                    if 5 < abs(self.angle) <= 0:
                        self.move(self.right_thruster, proportional_thrust)   #Setting value to be between 90 to 180
                        self.move(self.left_thruster, proportional_thrust)
                        self.move(self.bow_thruster, 90)
                        time.sleep(0.1)
                    
                    if  5 < abs(self.angle) <= 90 : # Forward
                        self.move(self.bow_thruster, proportional_angle)
                        self.move(self.right_thruster, proportional_thrust)   #Setting value to be between 90 to 180
                        self.move(self.left_thruster, proportional_thrust) 
                        time.sleep(0.1)

                    if 90 < abs(self.angle):    # Backward
                        if self.angle > 0:
                            self.move(self.bow_thruster, 120)
                            self.move(self.right_thruster, 90)
                            self.move(self.left_thruster, 90)
                            time.sleep(0.1)
                        elif self.angle < 0 :
                            self.move(self.bow_thruster, 70)
                            self.move(self.right_thruster, 90)
                            self.move(self.left_thruster, 90)
                            time.sleep(0.1)

                # "strong" (more than 6)
                elif 6 < self.dist:
                    if 5 < abs(self.angle) <= 0:
                        self.move(self.right_thruster, 150)
                        self.move(self.left_thruster, 150)
                        self.move(self.bow_thruster, 90)
                        time.sleep(0.1)

                    if 5 < abs(self.angle) <= 90:
                        self.move(self.bow_thruster, proportional_angle)
                        self.move(self.right_thruster, 150)
                        self.move(self.left_thruster, 150)
                        time.sleep(0.1)


                    if 90 < abs(self.angle):    # Backward
                        if 5 < self.angle:
                            self.move(self.bow_thruster, 120)
                            self.move(self.right_thruster, 90)
                            self.move(self.left_thruster, 90)
                            time.sleep(0.1)

                        if self.angle <  5:
                            self.move(self.bow_thruster, 70)
                            self.move(self.right_thruster, 90)
                            self.move(self.left_thruster, 90)
                            time.sleep(0.1)
if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    b = state_machine()

#test value
#state_machine.move("right_thruster", 40)