#! /usr/bin/python

import cv2 as cv
import numpy as np
import rospy



cap = cv.VideoCapture(0)


i = 0;
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Our operations on the frame come here
    hls = cv.cvtColor(frame,cv.COLOR_BGR2HLS)

    lower = np.array([150,220,0])
    upper = np.array([170,250,255])

    mask = cv.inRange(hls, lower, upper)
    res = cv.bitwise_and(frame,frame, mask= mask)

    

    ret2,thresh = cv.threshold(mask,127,255,0)
    tes,contours,hierarchy = cv.findContours(thresh, 1, 2)
    if contours :
        cnt = contours[0]
        M = cv.moments(cnt)
    
        (x,y),radius = cv.minEnclosingCircle(cnt)
        center = (int(x),int(y))
        radius = int(radius)
        cv.circle(mask,center,radius,(0,255,0),2)
    else:
        x=0
        y=0
    print("x=",x,"y=",y)

    # Display the resulting frame
    #cv.imshow('frame',frame)
    cv.imshow('frame',frame)
    cv.imshow('frame2',tes)
    cv.imshow('hand2',mask)
    #cv.imshow('hand3',laplacian)


    if cv.waitKey(1) & 0xFF == ord('q'):
        break
    
# When everything done, release the capture
cap.release()
#cap2.release()
cv.destroyAllWindows()





