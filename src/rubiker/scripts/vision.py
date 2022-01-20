#!/usr/bin/env python3
import rospy # Python library for ROS
from std_msgs.msg import Empty, String
#from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import sys
import tflite_runtime.interpreter as tflite
import kociemba

#import model
interpreter = tflite.Interpreter(model_path="/home/ubuntu/rubiker/model.tflite")
interpreter.allocate_tensors()
tmp = interpreter.get_input_details()
interpreter_in_idx = tmp[0]['index']
tmp = interpreter.get_output_details()
interpreter_out_idx = tmp[0]['index']


face_requested = False

def cbFace(msg):
  global face_requested
  print("face requested")
  face_requested = True

def main(gamma, coords):
  rospy.init_node('vision')
  global face_requested

  pub = rospy.Publisher('instructions', String, queue_size=1, latch=True)
  sub = rospy.Subscriber('getface', Empty, cbFace)

  cap = cv2.VideoCapture(0)
  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)


  rate = rospy.Rate(5);
 # br = CvBridge()
  lookUpTable = np.empty((1,256), np.uint8)
  for i in range(256):
    lookUpTable[0,i] = np.clip(pow(i / 255.0, gamma) * 255.0, 0, 255)

  color_to_pos = ['', '', '', '', '', '']
  colors = [[0 for j in range(9)] for i in range(6)]

  pub.publish("1");

  pos = ['U', 'R', 'F', 'D', 'L', 'B']
  for f in range(6):
    rospy.loginfo("Waiting for face request")
    while not face_requested and not rospy.is_shutdown():
      rate.sleep()

    if rospy.is_shutdown():
      return
    rospy.loginfo("Face requested")

    success = False
    while not success:

      success, bgr_frame = cap.read()

      if success:
        rospy.loginfo("Face captured")
        # Gamma correction
        bgr_frame = cv2.LUT(bgr_frame, lookUpTable)
        # Capture facelets
        for i in range(9):
          #frame, center_coords, radius, color, thickness
          dbg_frame = cv2.circle(bgr_frame, coords[i], 5, (100,255,100), 2)
          hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
          px_hsv = hsv_frame[coords[i][1], coords[i][0], :]
          px_hsv[0] /= 360.0
          px_hsv[1] /= 255.0
          px_hsv[2] /= 255.0

          interpreter_in = np.array([px_hsv], dtype='float32')
          interpreter.set_tensor(interpreter_in_idx, interpreter_in)
          interpreter.invoke()
          interpreter_out = interpreter.get_tensor(interpreter_out_idx)
          color = np.argmax(interpreter_out)
          colors[f][i] = color +1

          if i == 4:
            color_to_pos[color] = pos[f]

        face_requested = False

      else:
        rospy.loginfo("retrying face capture")

    rospy.loginfo(colors[f])
    pub.publish("1")

    # Save the image file
    # cv2.imwrite("/home/ubuntu/frame.jpg",dbg_frame)

  kociemba_input = ""
  for f in range(6):
    for i in range(8, -1, -1):
      kociemba_input += color_to_pos[colors[f][i]-1]

  kociemba_output =  kociemba.solve(kociemba_input)
  kociemba_output = kociemba_output.split(" ")

  instructions = ""
  for ins in kociemba_output:
    if ins == 'R2':
      instructions += "RR"
    elif ins == 'L2':
      instructions += "LL"
    elif ins == 'F2':
      instructions += "FF"
    elif ins == 'B2':
      instructions += "BB"
    elif ins == 'U2':
      instructions += "UU"
    elif ins == 'D2':
      instructions += "DD"
    elif ins == "L'":
      instructions += "l"
    elif ins == "R'":
      instructions += "r"
    elif ins == "F'":
      instructions += "f"
    elif ins == "B'":
      instructions += "b"
    elif ins == "U'":
      instructions += "u"
    elif ins == "D'":
      instructions += "d"
    else:
      instructions += ins

  instructions.append("P")
  pub.publish(instructions)

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
