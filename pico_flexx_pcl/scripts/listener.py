#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

from time import sleep
import serial
ser = serial.Serial('/dev/ttyACM0', 9600) # Establish the connection on a specific port
# ser.timeout = 2
counter = 1# Below 32 everything in ASCII is gibberish


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    ser = serial.Serial('/dev/ttyACM0', 9600)
    distance = data.data
    # print distance
    distance = distance*10
    # print distance
    if distance < 1:
        print "close"
        ser.write(str((1)))
    else:
        ser.write(str((2)))
        print "go_near"
    ser.close()
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("output", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    ser.close()

if __name__ == '__main__':
    listener()


# # odd_check = 1
# while True:
#     odd_check = odd_check + 1
    
#     if(odd_check%2 == 0):
#         ser.write(str((counter)))
#     else:
#         ser.write(str(0))

#     print ser.readline()
#     sleep(0.1)  

# ser.close()