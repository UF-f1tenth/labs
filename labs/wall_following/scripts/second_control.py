#!/usr/bin/env python
from __future__ import print_function
import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
from race.msg import pid_input
import math
import numpy as np
import sys
import os
from time import sleep

# TODO: modify these constants to make the car follow walls smoothly.
kp = 25.0
kd = 0.15
kp_vel = 25.0
kd_vel = 0.10
ki = 0.0
#servo_offset = 5.0*math.pi/180
#prev_error = 0.0
error = 0.0
integral = 0.0
vel_input = 1.0
prev_error = 0.0
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64

def control_callback(data):
        global integral
        global vel_input
	global prev_error
	global kp, kd, servo_offset
        velocity = data.pid_vel
        angle = 0.0
        error = (5*data.pid_error)
	print('\r',' Error = %f' %(error))
        if error!=0.0:     
                control_error = kp*error + kd*(error - prev_error)
                angle = angle + control_error*np.pi/180

		control_error_vel = kp_vel*error + kd_vel*(error - prev_error)
		# print "Control error velocity",control_error_vel

		# velocity = velocity - abs(control_error_vel)/10
		velocity = velocity + abs(control_error_vel)
        
	print('\r','prev_error = %f'%prev_error)
	print('\r','control_error =%f control_error_vel=%f' %(control_error,control_error_vel))
	print('\r','kp=%f and kd=%f'%(kp,kd))
	print('\r','Angle =%f and velocity =%f'%(angle*180/np.pi, velocity))
	prev_error = error
        if angle > 17*np.pi/180:
                angle = 17*np.pi/180
        if angle < -17*np.pi/180:
                angle = -17*np.pi/180

        msg = drive_param()
        base_velocity = 1.2

	
        if angle >= 8*np.pi/180  or angle <= -8*np.pi/180:
                velocity = 0.5 *base_velocity

        if (angle >0 and angle < 8*np.pi/180) or (angle <0 and angle > -8*np.pi/180):
                velocity = 0.8 * base_velocity

        if angle >= -1*np.pi/180 and angle <= 1*np.pi/180:
                velocity = velocity #No change

        if velocity < 0:
                velocity = 0

        if velocity > base_velocity:
                velocity = base_velocity
	print('\r',' Angle: %f Velocity: %f' %(angle*180/np.pi, velocity))
	
	#print('\r',' Error Control: %d\tControl %d\tAngle in Degrees: %d\tVelocity: %f\tAngle: %f' % (error, control_error, (angle*180/np.pi), velocity, angle))

        msg.velocity = velocity
        msg.angle = angle
        pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
	rospy.init_node('pid_controller_node', anonymous=True)
	rospy.Subscriber("pid_error", pid_input, control_callback)
	rospy.spin()
	

