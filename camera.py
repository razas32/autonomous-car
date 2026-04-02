import cv2
import numpy as np
import math

cap = cv2.VideoCapture(0)
StepSize = 5

# splits list l into chunks of size n
def getChunks(l, n):
    """Yield successive n-sized chunks from l."""
    a = []
    for i in range(0, len(l), n):
        a.append(l[i:i + n])
    return a

def process_camera_frame():
    ret,frame = cap.read()

    # flip vertically, filter noise, detect edges
    img = cv2.flip(frame, 0)
    blur = cv2.bilateralFilter(img,9,40,40)
    edges = cv2.Canny(blur,50,100)
    img_h = img.shape[0] - 1
    img_w = img.shape[1] - 1

    # scan each column from the bottom, grab first edge pixel found
    EdgeArray = []
    for j in range(0,img_w,StepSize):
        pixel = (j,0)
        for i in range(img_h-5,0,-1):
            if edges.item(i,j) == 255:
                pixel = (j,i)
                EdgeArray.append(pixel)
                break

    # if no edges found, nothing to do
    if len(EdgeArray) != 0:
        chunks = getChunks(EdgeArray, math.ceil(len(EdgeArray)/3))
    else:
        return

    #c = []
    distance = []
    for i in range(len(chunks)):
        x_vals = []
        y_vals = []
        for (x,y) in chunks[i]:
            x_vals.append(x)
            y_vals.append(y)
        avg_x = int(np.average(x_vals))
        avg_y = int(np.average(y_vals))
        #c.append([avg_y,avg_x])
        # distance from bottom-center of frame to mean point of this chunk
        distance.append(math.sqrt((avg_x - 320)**2 + (avg_y - 640)**2))
        cv2.line(img, (320, 640), (avg_x,avg_y), (0,0,255), 2)

    cv2.imshow("frame", img)
    cv2.waitKey(5)

    # shortest distance = least obstruction = preferred direction
    if(distance[0] < distance[1]):
        if(distance[0] < distance[2]):
            return 0
        else:
            return 2
    else:
        if(distance[1] < distance[2]):
            return 1
        else:
            return 2

while(1):
    ret = process_camera_frame()
    if ret == 0:
        print('Left direction is preferred');
    elif ret == 1:
        print('Forward direction is preferred');
    elif ret == 2:
        print('Right direction is preferred');
