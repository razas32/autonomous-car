#runs on the raspberry pi, its job is to look with the camera, listen with the ultrasonic sensor
#make decisions and send the into to FMU 
import RPi.GPIO as GPIO
import cv2
import numpy as np
import math
import time
from pymavlink import mavutil

# ultrasonic sensor pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24

# open camera
cap = cv2.VideoCapture(0)
StepSize = 5 #when scanning image check every 5 columns 

# set up GPIO pins for ultrasonic
def ultrasonic_setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

# fire the sensor and return distance in cm
def get_distance():
    # send a 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    # 30ms timeout = ~5m max range, returns 999 if no echo
    timeout = 0.03
    loop_start = time.time()

    # wait for echo to go high (sound left the sensor)
    StartTime = time.time()
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        if time.time() - loop_start > timeout:
            return 999.0

    # wait for echo to go low (sound came back)
    StopTime = time.time()
    loop_start = time.time()
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        if time.time() - loop_start > timeout:
            return 999.0

    # distance = (time * speed of sound) / 2 (divide by 2 because there and back)
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
    return distance

# splits a list into chunks of size n
    #used later to split camera edge points into left, middle and right section 
def getChunks(l, n):
    """Yield successive n-sized chunks from l."""
    a = []
    for i in range(0, len(l), n):
        a.append(l[i:i + n])
    return a

# reads a frame and returns preferred direction: 0=left, 1=forward, 2=right
def get_direction():
    ret, frame = cap.read() # read one frame from camera 
    if not ret:
        return None

    # flip, filter noise, detect edges
    img = cv2.flip(frame, 0) #flip image vertically 
    blur = cv2.bilateralFilter(img, 9, 40, 40) 
    edges = cv2.Canny(blur, 50, 100)
    img_h = img.shape[0] - 1
    img_w = img.shape[1] - 1

    # scan each column from bottom up, grab the first edge pixel found
    #objects closer to the car show uo lower in the image 
    EdgeArray = []
    for j in range(0, img_w, StepSize):
        pixel = (j, 0)
        for i in range(img_h - 5, 0, -1):
            if edges.item(i, j) == 255:
                pixel = (j, i)
                EdgeArray.append(pixel)
                break

    if len(EdgeArray) == 0: #no edges found
        return None

    # split edge pixels into 3 vertical chunks (left, center, right)
    chunks = getChunks(EdgeArray, math.ceil(len(EdgeArray) / 3))

    #c = []
    distance = []
    for i in range(len(chunks)): #for each chunk 1 find everage point 
        x_vals = []
        y_vals = []
        for (x, y) in chunks[i]:
            x_vals.append(x)
            y_vals.append(y)
        avg_x = int(np.average(x_vals))
        avg_y = int(np.average(y_vals))
        #c.append([avg_y, avg_x])
        
        # distance from bottom-center of frame to mean edge point of this chunk
        distance.append(math.sqrt((avg_x - 320) ** 2 + (avg_y - 640) ** 2))
        cv2.line(img, (320, 640), (avg_x, avg_y), (0, 0, 255), 2)

    cv2.imshow("frame", img)
    cv2.waitKey(5)

    if len(distance) < 3: #if there are not 3 proper chunks return none 
        return None

    # shortest distance = least obstruction = preferred direction
    if distance[0] < distance[1]:
        if distance[0] < distance[2]:
            return 0  # left
        else:
            return 2  # right
    else:
        if distance[1] < distance[2]:
            return 1  # forward
        else:
            return 2  # right

# setup ultrasonic
ultrasonic_setup()

# connect to FMU over USB, baud must match mavlink start command on FMU
the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=2000000)

the_connection.wait_heartbeat() #wait until FMU sends a heartbeat 
#print info showing connection worked 
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system,
      the_connection.target_component))

while True:
    dist = get_distance()

    direction = get_direction()
    if direction is None:
        direction = 1  # default to forward if camera gives nothing

    # pack both values into one MAVLink debug message
    # ind = direction, value = distance in cm
    message = mavutil.mavlink.MAVLink_debug_message(0, direction, dist)
    #sends to FMU 
    the_connection.mav.send(message)

    print("Sent -> direction: %d  distance: %.1f cm" % (direction, dist))
    time.sleep(0.1)  # 10 Hz
