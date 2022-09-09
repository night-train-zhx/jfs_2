#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import std_msgs.msg
import Jetson.GPIO as GPIO
import time
p=0
def callback(data):
    if(data.data):
        p.ChangeDutyCycle(50)
    else:
        p.ChangeDutyCycle(0)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global p
    rospy.init_node('Beeper', anonymous=False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    GPIO.setup(33, GPIO.OUT,initial=GPIO.LOW) #BEEPER
    p = GPIO.PWM(33, 2500)
    p.start(0)
    rospy.Subscriber('/beeper', std_msgs.msg.Bool, callback)
    rospy.spin()
    p.stop()
    GPIO.cleanup()

if __name__ == '__main__':
    listener()