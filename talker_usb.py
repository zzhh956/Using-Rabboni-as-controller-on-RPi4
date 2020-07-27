#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import traceback as tb
import math
import time

from rabboni import Rabboni

rab = Rabboni()
rab.connect()

rab.set_sensor_config(2, 500, 5, 100)
position_lr = 0
position_fb = 0
timer = time.time()
preW_fb = 0
preW_lr = 0

def usb_custom_callback(status): 
    global position_lr
    global position_fb
    global timer
    global preW_fb
    global preW_lr
    value = 0
    ratio = 0.7

    if(abs(preW_fb-status['Gyr'][0]) > 0):
        position_fb = (1 - ratio) * (position_fb + (status['Gyr'][0] * math.pi/180) * (time.time() - timer)) + ratio * status['Acc'][1]
    if(abs(preW_lr-status['Gyr'][1]) > 0):
        position_lr = (1 - ratio) * (position_lr + (status['Gyr'][1] * math.pi/180) * (time.time() - timer)) + ratio * status['Acc'][0]
    
    #Left and Right
    if position_lr < -0.35:
        s = 'right'
        value = status['Acc'][0]
    elif position_lr > 0.35:
        s = 'left'
        value = status['Acc'][0]
    #Forward and Backward
    elif position_fb < -0.4:
        s = 'forward'
        value = status['Acc'][1]
    elif position_fb > 0.4: 
        s = 'backward'
        value = status['Acc'][1]
    else:
        s = 'stable'

    pub1.publish(s)
    pub2.publish(value)
    print(s + ' ' + str(value))
    timer = time.time()
    preW_fb = position_fb
    preW_lr = position_lr

if __name__ == '__main__':
    rospy.init_node('talker', anonymous=True)
    pub1 = rospy.Publisher('command', String, queue_size=10)
    pub2 = rospy.Publisher('value', Float32, queue_size=10)

    try:
        rab.start_fetching_status(custom_callback=usb_custom_callback)
        rab.polling_status()
    except AssertionError:  # 結束程式
        print('Bye~!!')
    except Exception:
        tb.print_exc()
    finally:
        rab.disconnect()
