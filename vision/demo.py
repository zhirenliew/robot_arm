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

    frame = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
         
    # constantly show new frame
    cv.imshow("frame",frame)
    if cv.waitKey(1) == ord("q"):
        break

cv.destroyAllWindows()
