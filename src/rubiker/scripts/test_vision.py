#!/usr/bin/env python3
import rospy # Python library for ROS
import cv2
import numpy as np
import sys

def main(gamma, coords):
  rospy.init_node('test_vision')


  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

  lookUpTable = np.empty((1,256), np.uint8)
  for i in range(256):
    lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)

  input("Press ENTER to take another picture between each take, or CTRL+C to stop the program. Press ENTER now to continue.")

  while (not rospy.is_shutdown()):
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

      cv2.imwrite("/home/ubuntu/frame.jpg",dbg_frame)

    else:
      rospy.logwarn("Face capture FAILED. Try again")

    input()


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
