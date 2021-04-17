import cv2
import numpy as np
import imutils
from imutils.object_detection import non_max_suppression

class PedestrianDetection:

    def __init__(self):
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            #self.tracker = cv2.TrackerMOSSE_create()    # high speed, low accuracy
            #self.tracker = cv2.TrackerCSRT_create()      # low speed, high accuracy
            self.found = False
            self.bbox = ()

    def drawBox(self, img):
        x, y, w, h = int(self.bbox[0]), int(self.bbox[1]), int(self.bbox[2]), int(self.bbox[3])
        cv2.rectangle(img, (x,y), ((x+w),(y+h)), (255, 0, 255), 3, 1)

    def detectPedestrian(self, img):
        pedDetected = False
        # while not found:
        img = imutils.resize(img, width=min(400, img.shape[1]))
        #if self.found is False:
        (regions, _) = self.hog.detectMultiScale(img, winStride=(4, 4),padding=(4, 4),scale=1.05)
        print(regions)
        for(x, y, w, h) in regions:
            if w*h > 20000:
                cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                print("PedestrianDetected")
                print("Square is: ", w*h)
                pedDetected = True
            # self.bbox = tuple(map(tuple, regions))
        # print(type(self.bbox))
        # print(self.bbox)
        regions = np.array([[x, y, x + w, y + h] for (x, y, w, h) in regions])
        pick = non_max_suppression(regions, probs=None, overlapThresh=0.65)
        for(xA, yA, xB, yB) in pick:
            cv2.rectangle(img, (xA, yA), (xB, yB), (0, 255, 0), 2)

        #     if len(self.bbox) > 0:
        #         # self.bbox = tuple(map(tuple, pick)
        #         self.found = True
        #
        # else:
        #     self.trackPedestrian(img)

        cv2.imshow("Pedestrian detect", img)
        return pedDetected 

    def trackPedestrian(self, img):
        timer = cv2.getTickCount()

        self.tracker.init(img, self.bbox)

        success, self.bbox = self.tracker.update(img)

        if success:
            self.drawBox(img)

        else:
            cv2.putText(img, "Lost", (75,75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)

        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        cv2.putText(img, str(int(fps)), (75,50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 3)
        cv2.putText(img, "Tracking", (75, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 3)
        cv2.imshow("Tracking" , img)
