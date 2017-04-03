#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

# # Simple talker demo that published std_msgs/Strings messages
# # to the 'chatter' topic

import rospy
from geometry_msgs.msg import Twist  # import Twist messages 
from omgtools import *
import time
import numpy as np

def ridgeback_move_pub():  # publisher name
	# create vehicle
	vehicle = Holonomic()
	vehicle.set_options({'safety_distance': 0.1})
	vehicle.set_options({'ideal_prediction': False})
	vehicle.set_initial_conditions([0, 0]) #vehicle.set_initial_conditions([-1.5, -1.5])
	vehicle.set_terminal_conditions([2., 2.])

	# create environment
	environment = Environment(room={'shape': Rectangle(10., 5.)})
	rectangle = Rectangle(width=3., height=0.2)

	environment.add_obstacle(Obstacle({'position': [-2.1, 0.5]}, shape=rectangle))
	environment.add_obstacle(Obstacle({'position': [1.7, 0.5]}, shape=rectangle))

	# Obstacle
	#trajectory = {'velocity': {'time': [3., 4.], 'values': [[0.0, 0.0], [0., 0.0]]}}
	
	#simulation = {'trajectories': trajectory}
	
	#environment.add_obstacle(Obstacle({'position': [0.0, 0.0]}, shape=Circle(0.4), simulation=simulation))
	
	options = {}
	options['codegen'] = {'build': None}
	
	# create a point-to-point problem
	problem = Point2point(vehicle, environment, options, freeT=False)
	# problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
	problem.init()

	# create deployer
	update_time = 0.1
	sample_time = 0.1
	deployer = Deployer(problem, sample_time, update_time)

	# simulation of a motion planning application: go through 3 via points, while
	# an obstacle is changing position
	via_points = [[2., 2.]]	

	current_time = 0
	current_state = [0, 0]#current_state = [-1.5, -1.5]
	state_traj = np.c_[current_state]
	input_traj = np.c_[[0.0, 0.0]]

	n_samp = int(np.round(update_time / sample_time, 6))
	t00 = time.time()

	pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)  # Publisher('topic', 'topic type', queue_size)
	rospy.init_node('ridgeback_move', anonymous=True)  # initialize node 'ridgeback_move'
	rate = rospy.Rate(10)  # 100hz is dees publishing rate???
	target_reached = False
	saved = True
	while not rospy.is_shutdown():
		speed = Twist()  # make Twist object speed
		for via_point in (via_points):
			vehicle.set_terminal_conditions(via_point)
			
			vehicle.set_initial_conditions(via_point)  # for init guess
			deployer.reset()  # let's start from new initial guess
			while not target_reached:
				t0 = time.time() - t00
				if (t0 - current_time - update_time) >= 0.:
					current_time = t0
					# 'measure' current state (here ideal trajectory following is simulated)
					if state_traj.shape[1] > 1:
						current_state = state_traj[:, -n_samp - 1]
					else:
						current_state = state_traj[:, 0]
					# update motion planning
					trajectories = deployer.update(current_time, current_state)
			
					# store state & input trajectories -> simulation of ideal trajectory following
					state_traj = np.c_[state_traj, trajectories['state'][:, 1:n_samp + 1]]
					input_traj = np.c_[input_traj, trajectories['input'][:, 1:n_samp + 1]]
					x_pos = state_traj[0, -1]
					y_pos = state_traj[1, -1]
					x_speed = input_traj[0, -1]
					y_speed = input_traj[1, -1]
			
					# publish speed
					speed.linear.x = x_speed
					speed.linear.y = y_speed
					rospy.loginfo(speed)
					pub.publish(speed)
					rate.sleep()

					# check target
					if (np.linalg.norm(via_point - state_traj[:, -1]) < 5e-2 and np.linalg.norm(input_traj[:, -1]) < 5e-2):
						target_reached = True
						print('Target reached, close enough!')
					if (problem.iteration > 300):
						target_reached = True
						print('Target reached, max iterations...')

		if not saved:
			xpos = np.array(state_traj[0,:])
			ypos = np.array(state_traj[1,:])
			xspeed = np.array(input_traj[0,:])
			yspeed = np.array(input_traj[1,:])
			np.savetxt('xpos.csv', xpos, delimiter=",")
			np.savetxt('ypos.csv', ypos, delimiter=",")
			np.savetxt('xspeed.csv', xspeed, delimiter=",")
			np.savetxt('yspeed.csv', yspeed, delimiter=",")
			saved = True

if __name__ == '__main__':
	try:
		ridgeback_move_pub()
	except rospy.ROSInterruptException:
		pass

