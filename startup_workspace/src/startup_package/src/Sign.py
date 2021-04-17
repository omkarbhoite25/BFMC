#!/usr/bin/python
import rospy
import numpy as np
import time
import cv2
import os
from bfmclib.camera_s import CameraHandler
import matplotlib.pyplot as plt
from bfmclib.HelperFunctions import HelperFunctions as hf
from bfmclib.Mask import Mask
from std_msgs.msg import String

class Sign(object):

  def __init__(self):
    self.net = cv2.dnn.readNetFromDarknet("/home/alberto/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/bfmclib/yolov10_tiny-custom.cfg", r"/home/alberto/Documents/BFMC_Simulator/startup_workspace/src/startup_package/src/bfmclib/weights/yolov3_tiny-custom_20000.weights")

    self.size_thr = 2300

    ### Change here for custom classes for trained model

    self.LABELS = ['empty', 'Cross_walk', 'No_Entry', 'Highway_end', 'Parking_Spot',
                   'Priority_road', 'Stop', 'Highway_Start', 'Roundabout_mandatory', 'Traffic_Sign']
    np.random.seed(42)
    self.COLORS = np.random.randint(
        0, 255, size=(len(self.LABELS), 3), dtype="uint8")

    self.pub = rospy.Publisher('signs', String, queue_size=1)
    rospy.init_node('Sign_Detector', anonymous=True)
    cam = CameraHandler()
    while 1:
      frame = cam.getImage()
      if len(frame.shape) > 2 and frame.shape[2] == 3:
        frame = self.detection(frame)
        # cv2.imshow("Frame preview", self.detection(frame))
        cv2.imshow("sign_detector_node", frame)
      key = cv2.waitKey(1)
      
      if key == ord('q'):
        cv2.destroyAllWindows()
        break


  def get_detections(self, img):
    image = img
    (H, W) = image.shape[:2]

    ln = self.net.getLayerNames()
    ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    blob = cv2.dnn.blobFromImage(
        image, 1 / 255.0, (416, 416), swapRB=True, crop=False)
    self.net.setInput(blob)
    start = time.time()
    layerOutputs = self.net.forward(ln)
    end = time.time()

    # print("Frame Prediction Time : {:.6f} seconds".format(end - start))

    boxes = []
    confidences = []
    classIDs = []

    for output in layerOutputs:
      for detection in output:
        scores = detection[5:]
        classID = np.argmax(scores)
        confidence = scores[classID]

        if confidence > 0.1:
          box = detection[0:4] * np.array([W, H, W, H])
          (centerX, centerY, width, height) = box.astype("int")

          x = int(centerX - (width / 2))
          y = int(centerY - (height / 2))

          boxes.append([x, y, int(width), int(height)])
          confidences.append(float(confidence))
          classIDs.append(classID)
          if confidence > 0.98 and (width*height) > self.size_thr and x > W/2:
            print(self.LABELS[classID], "confidence", confidence, "Area", width*height)
            self.pub.publish(self.LABELS[classID])

    idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)
    return idxs, boxes, confidences, classIDs

  def detection(self, img):
    idxs, boxes, confidences, classIDs = self.get_detections(img)
    (H, W) = img.shape[:2]
    if len(idxs) > 0:

      for i in idxs.flatten():
        (x, y) = (boxes[i][0], boxes[i][1])
        (w, h) = (boxes[i][2], boxes[i][3])

        color = [int(c) for c in self.COLORS[classIDs[i]]]
        cv2.rectangle(img, (x, y), (x + w, y + h), color, 2)

        text = "{}: {:.4f}".format(self.LABELS[classIDs[i]], confidences[i], )
        cv2.putText(img, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,0.5, color, 2)
        col = (0,255,0) if (w*h) > self.size_thr else (0,0,250)
        cv2.putText(img, "is_near: {}".format((w*h) > 2500), (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX,0.5, col, 2)
        col = (0,255,0) if x > W/2 else (0,0,250)
        cv2.putText(img, "this_lane: {}".format(x > W/2), (x, y - 40), cv2.FONT_HERSHEY_SIMPLEX,0.5, col, 2)

    return img

if __name__ == "__main__":
  Sign()

	
	#print(lane_lines)
	# lane = mf()
	# lanekeep = lane.lane_keeping(masked_img,frame)
	#cv2.imshow("lane",lane_lines) 
