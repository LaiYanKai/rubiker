#!/usr/bin/env python3
import rospy # Python library for ROS
from std_msgs.msg import Empty, String
#from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import sys
import tflite_runtime.interpreter as tflite

face_requested = True

def cbFace(msg):
  global face_requested
  face_requested = True

def main(gamma, coords):
  rospy.init_node('vision')
  global face_requested

  pub = rospy.Publisher('facelets', String, queue_size=1)
  sub = rospy.Subscriber('getface', Empty, cbFace)

  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


  rate = rospy.Rate(5);
 # br = CvBridge()

  while not rospy.is_shutdown():
    if face_requested:
      rospy.loginfo("Face requested and captured")
      for i in range(10):
        success, bgr_frame = cap.read()
        if success:

        # Gamma correction
          lookUpTable = np.empty((1,256), np.uint8)
          for i in range(256):
            lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)
          bgr_frame = cv2.LUT(bgr_frame, lookUpTable)

        #frame, center_coords, radius, color, thickness
          for i in range(9):
            dbg_frame = cv2.circle(bgr_frame, coords[i], 5, (100,255,100), 2)
            hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
            px_hsv = hsv_frame[coords[i][1], coords[i][0], :]
            rospy.loginfo("\t" +str(px_hsv[0]) + "\t" + str(px_hsv[1]) + "\t" + str(px_hsv[2]))

          

        cv2.imwrite("/home/ubuntu/frame.jpg",dbg_frame)
        face_requested = False
    rate.sleep()

if __name__ == '__main__':
  try:
    coords = []
    for i in range(2,11):
      coord = sys.argv[i].split(",")
      coords.append((int(coord[0]), int(coord[1])))
    gamma = float(sys.argv[1])
    main(gamma, coords)

  except rospy.ROSInterruptException:
    pass
