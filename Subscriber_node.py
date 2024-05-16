#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
from ultralytics.utils import ASSETS
from ultralytics.models.yolo.detect import DetectionPredictor
import cv2
import time
from std_msgs.msg import Int32

VR = Int32()
VL = Int32()

topicNameVR='vr'
topicNameVL='vl'

bottle_x = 0
bottle_y = 0

def callback_recieved_position(msg):
    #Extract bottle positions from the received message
    global bottle_x, bottle_y
    bottle_x = msg.data[0]
    bottle_y = msg.data[1]
def main():
    rospy.init_node('bottle_position_reciever', anonymous=True)
    rospy.Subscriber('detected_bottle_position', Float32MultiArray, callback_recieved_position)
    publeftmot=rospy.Publisher(topicNameVL, Int32, queue_size=5)
    pubrightmot=rospy.Publisher(topicNameVR, Int32, queue_size=5)
    while not rospy.is_shutdown():
        msg = Float32MultiArray()
        #print("angular velocity: %s" %bottle_x)
        vr= 4
        vl= 4
        if(bottle_x < 850 and bottle_x!=0):
         vr= 6
         vl= 4
        elif(bottle_x > 850 and bottle_x!=0):
         vr= 4
         vl= 6
        else:
         vr= 4
         vl= 4

        VR.data = int(vr)
        VL.data = int(vl)        
            
        # Publish wheel speeds	
        publeftmot.publish(VL)
        pubrightmot.publish(VR)	
        print("angular velocity: %s" %vl) 
 
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass