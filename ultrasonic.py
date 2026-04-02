#Libraries
import RPi.GPIO as GPIO
import time

#set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24

def ultrasonic_setup():
    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    #set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # send a 10us pulse to trigger the sensor
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # wait for echo to go high (sound left the sensor)
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # wait for echo to go low (sound came back)
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # distance = (time * speed of sound) / 2, divide by 2 because there and back
    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2

    return distance

ultrasonic_setup();
while True:
    dist = distance()
    print ("Measured Distance = %.1f cm" % dist)
    time.sleep(0.1)
