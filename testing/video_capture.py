import numpy as np
import cv2

cap = cv2.VideoCapture(0)

# Define the codec and create VideoWriter object
fps=10.0
out = cv2.VideoWriter('output.avi',4, fps, (640,480))
i=0
while(cap.isOpened()):
    ret, frame = cap.read()
    if ret==True:
	i+=1
        # write the flipped frame
        out.write(frame)

        cv2.imshow('frame',frame)
    if i==20:
	break

# Release everything if job is finished
cap.release()
out.release()
cv2.destroyAllWindows()
