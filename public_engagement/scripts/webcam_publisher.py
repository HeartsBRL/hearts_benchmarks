#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import datetime

bridge = CvBridge()

def main():
    rospy.init_node('cam_republisher')
    
    #last_img_time = datetime.datetime.now()
    
    pub = rospy.Publisher('/roah_ipcam/image', Image)
    
    while (True):
        current_img_time = os.path.getmtime('/tmp/door_image.png')
        
        #if current_img_time > last_img_time:
        last_img_time = current_img_time
        
        try:
            cv2_img = cv2.imread('/tmp/door_image.png')
            img = bridge.cv2_to_imgmsg(cv2_img, "bgr8")
            
            pub.publish(img)
            print "ok!"
        
        except CvBridgeError, e:
            print(e)
            
        except:
            print("e")
        
        rospy.sleep(0.1)
    
    rospy.spin()
    

if __name__ == '__main__':
    main()
