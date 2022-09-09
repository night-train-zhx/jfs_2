#!/usr/bin/python
import rospy
import time
import RPi.GPIO as GPIO
from std_msgs.msg import String
from std_msgs.msg import Byte
pin22 = 22
pin24 = 24
pin26 = 26
pin19 = 19
pin21 = 21
pin23 = 23
pin18 = 18

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    rospy.loginfo( "I heard ")
    if(data.data==1):                       #turn on
        GPIO.output(pin18, GPIO.HIGH)
    if(data==2):
        GPIO.output(pin19, GPIO.LOW)
    if(data==3):
        GPIO.output(pin21, GPIO.LOW)
    if(data==4):
        GPIO.output(pin23, GPIO.LOW)
    if(data.data==5):                       #turn off
        GPIO.output(pin18, GPIO.LOW)
    if(data==6):
        GPIO.output(pin19, GPIO.HIGH)
    if(data==7):
        GPIO.output(pin21, GPIO.HIGH)
    if(data==8):
        GPIO.output(pin23, GPIO.HIGH)
        

def listener():
    GPIO.setup(pin19, GPIO.OUT)  # button pin set as input
    GPIO.setup(pin21, GPIO.OUT)
    GPIO.setup(pin23, GPIO.OUT)
    GPIO.setup(pin18, GPIO.OUT)

    GPIO.output(pin18, GPIO.LOW)
    GPIO.output(pin19, GPIO.HIGH)
    GPIO.output(pin21, GPIO.HIGH)
    GPIO.output(pin23, GPIO.HIGH)
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/chatter", Byte, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    GPIO.setmode(GPIO.BOARD)  # BOARD pin-numbering scheme
    listener()
