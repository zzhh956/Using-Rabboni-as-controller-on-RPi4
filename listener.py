#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import RPi.GPIO as GPIO
import time
import numpy as np

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)

pR=GPIO.PWM(32,50)# 50hz frequency
pL=GPIO.PWM(12,50)

pR.start(7)
pL.start(7)

pR.ChangeDutyCycle(7)
pL.ChangeDutyCycle(7)

command = 'forward'

def control(value):
    global command
    lr_strength = 1.4     #左右轉幅度，越大越強
    fb_strength = 1.3     #前後強度，越大越強
    
    print(fb_strength*value.data+7)
     
    if command == 'right':
        pR.ChangeDutyCycle(7-lr_strength)
        pL.ChangeDutyCycle(7+lr_strength)
    elif command == 'left':        
        pR.ChangeDutyCycle(7+lr_strength)
        pL.ChangeDutyCycle(7-lr_strength)
    else:
        pR.ChangeDutyCycle(fb_strength*value.data+7)  
        pL.ChangeDutyCycle(fb_strength*value.data+7)
    
def Command(data):
    global command
    command = data.data
    print(command)
        
def listener():    
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('command', String, Command)
    rospy.Subscriber('value' , Float32 , control)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print("STOP")
    finally:
        GPIO.cleanup()

