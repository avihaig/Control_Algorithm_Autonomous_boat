#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

RIGHT_THRUSTER_CODE = 1
LEFT_THRUSTER_CODE = 2
BOW_THRUSTER_CODE = 3
RIGHT_SERVO_CODE = 4
LEFT_SERVO_CODE = 5
data_list = [0,0,0,0,0]
#data_list = [right_thruster , left_thruster , bow_thruster , right servo , left servo]
def callback(data): #3180
    code = int(data / 1000) # = 3  
    command_value = data - code*1000  # 3180 - 3* 1000 = 180
    data_list[code-1] = command_value # code - 1 = 3 -1 = 2   
    
def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("simulation/publisher", Int32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()