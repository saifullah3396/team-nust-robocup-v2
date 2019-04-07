#!/usr/bin/python

import binascii
import cv2
import json
import sys
import rospy
from data_handler import DataHandler

DATA_BUFFER = 8096

if __name__ == '__main__':
    try:
      rospy.init_node('data_handler_node', anonymous=True)      
      #path = "../data/data.json";
      #json_data=open(path).read()
      handler = DataHandler()
      rate = rospy.Rate(10)
      #path_to_image = "/home/sensei/Pictures/1.jpeg"
      #image = cv2.resize(cv2.imread(path_to_image), (320, 240))
      #yuv = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
      #jpg = cv2.imencode('.jpg', yuv)[1].tostring()
      #image_data = binascii.hexlify(jpg)
      while not rospy.is_shutdown():
        #handler.parse_data(json_data)
        #handler.parse_image(image_data)
        rate.sleep()
    except rospy.ROSInterruptException:
      pass
