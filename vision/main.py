import cv2 as cv
import numpy as np
from picamera2 import Picamera2

cam = Picamera2()
cam.start()

while True:
    frame = cam.capture_array()
    if frame is None:
        print("no frame captured")
        break
         
    #frame = frame[:,:,::-1] # RGB2BGR 
    frame = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
    img = cv.cvtColor(frame,cv.COLOR_BGR2HSV)
    
    # mask colors
    mask1 = cv.inRange(img,(0,100,50),(5,255,255))
    mask2 = cv.inRange(img,(175,100,50),(179,255,255))
    img = cv.bitwise_or(mask1,mask2)
    
    # opening
    kernel = np.ones((3,3),np.uint8)
    img = cv.morphologyEx(img,cv.MORPH_OPEN,kernel)
    
    # find and draw contours
    contours,hierarchy = cv.findContours(img,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours,key=cv.contourArea)
        peri = cv.arcLength(c,True)
        approx = cv.approxPolyDP(c,0.015*peri,True)
        cv.drawContours(frame,[approx],-1,(0,0,150),3) # frame still in bgr form
    
    # constantly show new frame
    cv.imshow("frame",frame)
    if cv.waitKey(1) == ord("q"):
        break

cv.destroyAllWindows()
