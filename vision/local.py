import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("no frame read")
        break

         
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
        if len(approx) < 9:
            cv.drawContours(frame,[approx],-1,(0,0,150),3) # frame still in bgr form
    
    # constantly show new frame
    cv.imshow("frame",frame)
    if cv.waitKey(1) == ord("q"):
        break

cap.release()
cv.destroyAllWindows()



# TODO
# why do we *perimeter
