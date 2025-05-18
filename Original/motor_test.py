#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy, time
from xycar_msgs.msg import XycarMotor

motor = None
motor_msg = XycarMotor()

def drive(angle, speed):
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

rospy.init_node('driver')
motor = rospy.Publisher('xycar_motor', XycarMotor, queue_size=1)

while not rospy.is_shutdown():
    for i in range(50):
        drive(angle=50.0, speed=20.0)
        time.sleep(0.1)
        
    for i in range(60):
        drive(angle=-50.0, speed=20.0)
        time.sleep(0.1)
