#!/usr/bin/env python3
import rospy # Python library for ROS
import cv2
import numpy as np
import sys

def main(gamma, coords):
  rospy.init_node('test_vision')
  num_burst = 10
  
  f_csv = open("/home/ubuntu/data.csv", "w")

  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

  lookUpTable = np.empty((1,256), np.uint8)
  for i in range(256):
    lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)

  c = 0;
  while (True):
    rospy.loginfo(str(c) + " takes done. Key in color num and ENTER, or q  to stop the program")
    try:
      face_color = int(input())
    except:
      rospy.loginfo("Quiting...")
      return
    
    if (rospy.is_shutdown()):
    	break;
    rospy.loginfo("Capturing face color " + str(face_color) + " for " + str(num_burst) + " times")

    for b in range(num_burst):
      success, bgr_frame = cap.read()
      if success:
        # Gamma correction
        bgr_frame = cv2.LUT(bgr_frame, lookUpTable)

        # Capture facelets
        for i in range(9):
          #frame, center_coords, radius, color, thickness
          dbg_frame = cv2.circle(bgr_frame, coords[i], 5, (100,255,100), 2)
          hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
          px_hsv = hsv_frame[coords[i][1], coords[i][0], :]
          rospy.loginfo("\t" +str(px_hsv[0]) + "\t" + str(px_hsv[1]) + "\t" + str(px_hsv[2]))
          f_csv.write(str(px_hsv[0]) + "," + str(px_hsv[1]) + "," + str(px_hsv[2]) + "," + str(face_color) + "\n")

          cv2.imwrite("/home/ubuntu/frame.jpg",dbg_frame)
      else:
        rospy.logwarn("Face capture FAILED")
    c += 1

  f_csv.close()
  
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
