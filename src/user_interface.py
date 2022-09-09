#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import std_msgs.msg
import Jetson.GPIO as GPIO
import time
led_pin_1 = 19
led_pin_2 = 21
led_pin_3 = 23
pub=0
def callback(data):
    if(data.data&4):
        GPIO.output(led_pin_1, GPIO.LOW)
    else:
        GPIO.output(led_pin_1, GPIO.HIGH)
    if(data.data&1):
        GPIO.output(led_pin_2, GPIO.LOW)
    else:
        GPIO.output(led_pin_2, GPIO.HIGH)
    if(data.data&2):
        GPIO.output(led_pin_3, GPIO.LOW)
    else:
        GPIO.output(led_pin_3, GPIO.HIGH)

def key_process(channel):
    if(channel==35):
        key=1
    elif(channel==31):
        key=2
    elif(channel==29):
        key=3
    else:
        key=0
    msg = std_msgs.msg.UInt8()
    msg.data = key
    pub.publish(msg)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global pub
    rospy.init_node('UserInterface', anonymous=False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup([29,31,35], GPIO.IN) #buttons  
    GPIO.setup([led_pin_1,led_pin_2,led_pin_3], GPIO.OUT,initial=GPIO.HIGH)
    GPIO.add_event_detect(29, GPIO.RISING, callback=key_process, bouncetime=200)
    GPIO.add_event_detect(31, GPIO.RISING, callback=key_process, bouncetime=200)
    GPIO.add_event_detect(35, GPIO.RISING, callback=key_process, bouncetime=200)
    rospy.Subscriber('/leds', std_msgs.msg.UInt8, callback)
    pub = rospy.Publisher('/keys', std_msgs.msg.UInt8, queue_size=10)
    rospy.spin()
    GPIO.cleanup()

if __name__ == '__main__':
    listener()