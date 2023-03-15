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
        self.arduino = serial.Serial(port='/dev/ttyUSB0' , baudrate = 9600, timeout = 1)
        self.x_vel = 0
        self.y_vel = 0
        self.ang_z_vel = 0
        self.cmd_list = [0,0,0]
        self.vector_angle_degress = 0
        self.true_angle = 0
        self.servo_angle = 0
        self.servo_angle_command = 0
        self.thruster_factor = 0
        self.speed_vector_size = 0
        self.absolute_speed_vector_size =0
        self.PD_commands = [0,0,0,0,0,0]
        self.state = 0


        #added
        self.right_thruster = "right_thruster"
        self.left_thruster = "left_thruster"
        self.bow_thruster = "bow_thruster"
        self.right_servo = "right_servo"
        self.left_servo = "left_servo"
        self.decision_distance = 4.0
        self.too_close_distance = 0.8
        self.long_decision_angle = 45.0
        self.no_turn_long_angle = 5.0
        self.short_decision_angle = 30.0
        self.no_turn_short_angle = 10.0
        self.straight=90.0
        self.stop_motor=94.0
        self.stop_front_motor=94.0
        self.front_motor_correction = 105.0
        self.right_turn=70.0
        self.left_turn=155.0
        self.T=30.0
        self.K=0.2
        self.strong_motor = 45.0
        self.medium_motor = 60.0
        self.week_motor = 75.0
        self.crab_right_angle = 180.0
        self.crab_left_angle = 0.0 
        self.cannon_on = 5
        self.cannon_off = 6
        self.thrust_angle = 0
        self.x = 0
        self.y = 0
        self.z = 0
 
        while not rospy.is_shutdown():
            rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
            self.calculations()  
   

   
    def cmd_callback(self, cmd_data):
        self.x = float(cmd_data.linear.x)
        self.y = float(cmd_data.linear.y)
        self.z = float(cmd_data.angular.z)
        self.cmd_list = [self.x_vel, self.y_vel, self.ang_z_vel]
 
    def calculations(self):
        try:
            vector_angle_degress = ((math.atan(float(self.x)/float(self.y)))/math.pi)*180
            if (float(self.x) >= 0 and float(self.y) >= 0):
                true_angle = vector_angle_degress
            elif (float(self.x) >= 0 and float(self.y) < 0):
                true_angle = vector_angle_degress + 180
            elif (float(self.x) < 0 and float(self.y) >= 0):
                true_angle = vector_angle_degress + 360
            elif (float(self.x )< 0 and float(self.y) < 0):
                true_angle = vector_angle_degress + 180
        except ZeroDivisionError: 
            if (self.x>=0):
                true_angle = 90
            else:
                true_angle = 270
    ## calc distance 
        goal_dist = math.sqrt(self.x**2+self.y**2)

        no_turn = 0
        turn_while_sail = 0
    ## choose by distance ##
        if (goal_dist >= self.decision_distance):
            ## decide by angle
            if (abs(true_angle-self.straight) <= self.long_decision_angle and abs(true_angle-self.straight) >= self.no_turn_long_angle):
                turn_while_sail = 1
            if (abs(true_angle-self.straight)<self.no_turn_long_angle):
                no_turn = 1
        elif (goal_dist < self.decision_distance and goal_dist > self.too_close_distance):
            if (abs(true_angle-self.straight) <= self.short_decision_angle and abs(true_angle-self.straight) >= self.no_turn_short_angle):
                turn_while_sail = 1
            if (abs(true_angle-self.straight)<self.no_turn_short_angle):
                no_turn = 1
        elif (goal_dist < self.too_close_distance):
            no_turn = 1

    ## crab movment
        thrust_angle = self.straight
        if (self.z == 1):
            if (self.y>0):
                thrust_angle = self.crab_right_angle
            elif(self.y<0):
                thrust_angle = self.crab_left_angle  
        
        front_motor_turn = self.stop_motor
    ## front motor command
        if(self.y>0):
            front_motor_turn = self.right_turn
        elif(self.y<0):
            front_motor_turn = self.left_turn

    ## cannon control
        cannon_control = 0
        if (self.z == self.cannon_on): 
            cannon_control = self.cannon_on
        if (self.z == self.cannon_off):
            cannon_control = self.cannon_off

## move only forward
        if (no_turn):
            if (goal_dist >= self.decision_distance):
                new_motor_commands = np.array([self.strong_motor , self.strong_motor , self.front_motor_correction , thrust_angle , thrust_angle , cannon_control],dtype=np.float32)
            elif (goal_dist < self.decision_distance and goal_dist > self.too_close_distance):
                new_motor_commands = np.array([self.medium_motor , self.medium_motor , self.front_motor_correction , thrust_angle , thrust_angle , cannon_control],dtype=np.float32)
            elif (goal_dist < self.too_close_distance):
                new_motor_commands = np.array([self.week_motor , self.week_motor , self.stop_motor , thrust_angle , thrust_angle , cannon_control],dtype=np.float32)
    
## move forward and turn   
        elif(turn_while_sail):
            if (goal_dist >= self.decision_distance):
                new_motor_commands = np.array([self.strong_motor , self.strong_motor , front_motor_turn , thrust_angle , thrust_angle , cannon_control],dtype=np.float32)
            elif (goal_dist < self.decision_distance and goal_dist > self.too_close_distance):
                new_motor_commands = self.np.array([self.medium_motor , self.medium_motor , front_motor_turn , thrust_angle , thrust_angle , cannon_control],dtype=np.float32)
            elif (goal_dist < self.too_close_distance):
                new_motor_commands = np.array([self.week_motor , self.week_motor , front_motor_turn , thrust_angle , thrust_angle , cannon_control],dtype=np.float32)
        
## turn on the spot
        elif(self.y!=0):
            new_motor_commands = np.array([self.stop_motor , self.stop_motor , front_motor_turn, thrust_angle , thrust_angle, cannon_control])
        elif(self.x==0 and self.y==0):
            new_motor_commands = np.array([self.stop_motor , self.stop_motor , self.stop_front_motor  , thrust_angle , thrust_angle, cannon_control])

        if(self.z==8):
            new_motor_commands = np.array([75.0,115.0,94.0,90.0,90.0,5])
        print("move boat")
        print(new_motor_commands)
        self.send_commands(new_motor_commands)        
   

 
    def send_commands(self, command_list):
        print("sending:")
        print(command_list)
        self.move(self.right_thruster,command_list[0])
        self.move(self.left_thruster,command_list[1])
        self.move(self.bow_thruster,command_list[2])
        self.move(self.right_servo,command_list[3])
        self.move(self.left_servo,command_list[4])
        # self.move(self.right_thruster,command_list[6])   ## cannon control not implemented yet

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
        
 
   
        
       
 
if __name__ == '__main__':
  rospy.init_node('listener', anonymous=True)
  b = state_machine()