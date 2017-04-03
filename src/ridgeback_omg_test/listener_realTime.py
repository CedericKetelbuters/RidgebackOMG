#!/usr/bin/env python

'''
##############################################
# REMARKS
# robot pose: model_states_data.pose[3]
# walls poses: link_states_data.pose[2,3,4,5]
# cylinder pose: model_states_data.pose[1]
##############################################
'''

import rospy
from rospy.numpy_msg import numpy_msg
import std_msgs.msg
import sensor_msgs.msg
import gazebo_msgs.msg
import matplotlib.pyplot as plt
import numpy as np

#Global variables

xc = 0.0
yc = 0.0
radius = 0.0

#Initialize KF
x_est_k = np.zeros((4, 1),dtype=float)
x_est_k_1 = np.zeros((4, 1),dtype=float)
#B = np.array([0]) no input given to the cylinder
#z = np.array([0,0])
delta_t = 0.1
t_1 = 0.0
F = np.array([[1, delta_t, 0 ,0 ],[0, 1, 0, 0],[0, 0, 1, delta_t],[0, 0, 0, 1]], float)
H = np.array([[1, 0, 0, 0],[0, 0, 1, 0]], float)
P_k = np.zeros((4, 4),dtype=float)
P_k_1 = np.zeros((4, 4),dtype=float)
Q = 0.05 * np.identity(4, dtype = float)
R = 0.05 * np.identity(2, dtype = float)
S_k = np.zeros((2, 2),dtype=float)
K_k = np.zeros((2, 2),dtype=float)
I4 = np.identity(4, dtype = float)

def callback_scan(scan):
	global xc, yc, radius, x_est_k, delta_t, F, t_1
	delta_t = (scan.header.stamp.secs + 1e-9 * scan.header.stamp.nsecs) - t_1
	F = np.array([[1, delta_t, 0 ,0 ],[0, 1, 0, 0],[0, 0, 1, delta_t],[0, 0, 0, 1]], float)
	[xc, yc, radius] = estimate_obstacle(scan)
	x_est_k = KF(xc, yc, F)
	t_1 = scan.header.stamp.secs + 1e-9 * scan.header.stamp.nsecs


def estimate_obstacle(scan):
	### OBSTACLE ESTIMATION (xc, yc, R)
	nb_angle = (scan.angle_max - scan.angle_min) / scan.angle_increment #all [rad]
	angles = np.arange(scan.angle_min, scan.angle_max, scan.angle_increment)
	rob_theta = np.arccos(model_states_data.pose[3].orientation.w**2-model_states_data.pose[3].orientation.z**2)
	cartes_data = np.array([scan.ranges*np.cos(angles + rob_theta), scan.ranges*np.sin(angles + rob_theta)])

	# --> Position of the walls (2,4: vertical; 3,5: horizontal)
	wallx_left = link_states_data.pose[2].position.x
	wallx_right = link_states_data.pose[4].position.x
	wally_down = link_states_data.pose[3].position.y
	wally_up = link_states_data.pose[5].position.y

	notwall_points = (cartes_data[0,:] >= wallx_left+0.2) * (cartes_data[0,:] <= wallx_right-0.2) * (cartes_data[1,:] >= wally_down+0.2) * (cartes_data[1, :] <= wally_up-0.2)
	notwall_points = np.where(notwall_points)
	if notwall_points[0].size > 3:
		circ_points = cartes_data[:, notwall_points]
		x = circ_points[0][0]
		y = circ_points[1][0]
		b = -(x**2 + y**2)
		xyc = np.array([x,y,np.ones(x.shape)]).transpose()
		a = np.linalg.lstsq(xyc, b)[0]

		#Estimation has to take into account robot position
		xc = model_states_data.pose[3].position.x -.5*a[0]
		yc = model_states_data.pose[3].position.y -.5*a[1]
		radius  =  ((a[0]**2+a[1]**2)/4-a[2])**0.5

		#Calculate error in estimation (in cm)
		e_x = (xc - model_states_data.pose[1].position.x) * 100
		e_y = (yc - model_states_data.pose[1].position.y) * 100
		e_radius = (radius - 0.5) * 100
		#print [xc, yc, radius]
		#print ["Error in cm: ", e_x, e_y, e_radius]

		return [xc, yc, radius]
	else:
		print("No obstacle in the field of vision")

def KF(xc, yc, F):
	global x_est_k, x_est_k_1, S_k
	global P_k, P_k_1
	### KALMAN FILTER
	# --> Prediction
	x_est_k = np.dot(F, x_est_k_1)
	P_k = np.dot(np.dot(F,P_k_1),F.transpose()) + Q

	# --> Correction
	y_k = np.array([[xc], [yc]]) - np.dot(H, x_est_k)
	S_k = np.dot(H,np.dot(P_k, H.transpose())) + R
	K_k = np.dot(np.dot(P_k, H.transpose()), np.linalg.inv(S_k))
	x_est_k = x_est_k + np.dot(K_k, y_k)
	P_k = np.dot((I4 - np.dot(K_k,H)), P_k)    

	x_est_k_1 = x_est_k
	P_k_1 = P_k

	#Calculate error in estimation (in cm/s)
	e_vx = (0.1 - x_est_k[1]) * 100
	e_vy = (-0.1 - x_est_k[3]) * 100
	#print (x_est_k[1], x_est_k[3])
	#print ["Error in cm/s: ", e_vx, e_vy]

	return x_est_k

def callback_world(data2):
	global link_states_data
	link_states_data = data2

def callback_pose(data3):
	global model_states_data
	model_states_data = data3

def scan_subs_pub():
	### SUBSCRIBERS
	rospy.init_node('scan_listener', anonymous=False)
	rospy.Subscriber("scan", sensor_msgs.msg.LaserScan, callback_scan)
	rospy.Subscriber("gazebo/link_states", gazebo_msgs.msg.ModelStates, callback_world)
	rospy.Subscriber("gazebo/model_states", gazebo_msgs.msg.ModelStates, callback_pose)
	# spin() simply keeps python from exiting until this node is stopped

	### PUBLISHER
	#publishes [xc, yc, radius, vx, vy]
	pub = rospy.Publisher('obstacle_pose', std_msgs.msg.Float32MultiArray, queue_size=10)
	rate = rospy.Rate(5) # 5hz 
	while not rospy.is_shutdown():
		msg = std_msgs.msg.Float32MultiArray()
		msg.data = np.array([xc, yc, radius, x_est_k[1][0], x_est_k[3][0]])
		rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()

	#spin only if no publisher
	#rospy.spin()  

if __name__ == '__main__':
	try:
		scan_subs_pub()
	except rospy.ROSInterruptException:
		pass