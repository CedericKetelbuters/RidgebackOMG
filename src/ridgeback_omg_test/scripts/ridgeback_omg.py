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
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates  
from omgtools import *
import time
import numpy as np
from cv2 import circle
from mercurial.bundlerepo import instance #wat is dees
import tf
from gip_optimal_navigation.msg import obstacleMsg, obstacleMsg_vector
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1000)  # Publisher('topic', 'topic type', queue_size)
speed = Twist()
state = ModelStates()

class circle_obs:
        def __init__(self,x,y,r,x_vel,y_vel):
                self.x=x
                self.y=y
                self.r=r
                self.x_vel=x_vel
                self.y_vel=y_vel
class ridgeback_omg_control:
        def __init__(self):
                self.pose_x=0.0
                self.pose_y=0.0
                self.dyn_msg=0
                self.dyn_obstacles=[1]
                self.subscriber_pos = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_state) # Subscriber('topic', 'message type', callback)
                self.subscriber_obs = rospy.Subscriber('obstacle_pose',std_msgs.msg.Float32MultiArray,self.callback_dynamic)
                
                # create vehicle hier steeds aanpassen
                vehicle_options={'syslimit':'norm_2'}
                vehicle_bounds={'vmax':1.1}
                vehicle = Holonomic(Circle(.466),vehicle_options,vehicle_bounds) #Approximate Ridgeback as rectangle with orientation 0 :Rectangle(0.793,0.96)
                vehicle.set_options({'safety_distance': 0.1})
                vehicle.set_options({'ideal_prediction': False})
                vehicle.set_initial_conditions([0, 0]) #vehicle.set_initial_conditions([-1.5, -1.5])
                vehicle.set_terminal_conditions([2., 2.])
                self.vehicle=vehicle
                #initialize problem
                options = {}
                options['codegen'] = {'build': None}
                #initialize environment
                self.environment = Environment(room={'shape': Rectangle(20., 10.)})
                rectangle = Rectangle(width=3., height=0.2)
                self.stat_obs=[] #[Obstacle({'position': [-5, 1.0]}, shape=rectangle),Obstacle({'position': [-4, 1.0]}, shape=rectangle)]
                self.environment.add_obstacle(self.stat_obs)
                
   
                # create a point-to-point problem
                self.problem = Point2point(self.vehicle, self.environment, options, freeT=False)
                self.problem.init()
                # problem.set_options({'solver_options': {'ipopt': {'ipopt.linear_solver': 'ma57'}}})
                
                # create deployer
                self.update_time = 0.1
                self.sample_time = 0.01
                self.deployer = Deployer(self.problem,self.sample_time,self.update_time)
                
                self.via_points = [[2., 0.]] 
 
                self.current_time = 0
                self.current_state = [0, 0]#self.current_state = [-1.5, -1.5]
                state_traj = np.c_[self.current_state]
                input_traj = np.c_[[0.0, 0.0]]
 
                self.n_samp = int(np.round(self.update_time /self.sample_time, 6))
#                 self.i=0
                
        def callback_state(self,state):
                self.pose_x = state.pose[1].position.x
                self.pose_y = state.pose[1].position.y
                euler = tf.transformations.euler_from_quaternion((state.pose[1].orientation.x,state.pose[1].orientation.y,state.pose[1].orientation.z,state.pose[1].orientation.w))
                self.th = euler[2]
                self.vel_x=state.twist[1].linear.x
                self.vel_y=state.twist[1].linear.y 
        
                
        def callback_dynamic(self,dyn_obs):
                #msg.data = np.array([xc, yc, radius, x_est_k[1][0], x_est_k[3][0]])
                
                self.dyn_obstacles=[]
#                 for obs in dyn_obs:
#                         if obs[0]=='cylinder':
#                             self.dyn_obstacles.append(circle_obs(obs[1],obs[2],obs[3],obs[4],obs[5]))
#                         else:
#                             pass
#                 self.dyn_msg=1 #boolean to see if we need to update the obstacles in the loop, improve time
#                 #environment updaten in lus zelf
                if dyn_obs[0]=='cylinder1':
                     self.dyn_obstacles.append(circle_obs(dyn_obs[1],dyn_obs[2],dyn_obs[3],dyn_obs[4],dyn_obs[5]))
                
#       def delete_existing_dynamic_obstacle(self,number_dynamic_obstacles):
#               for i in range(number_dynamic_obstacles):
#                       environment.obstacles.pop() #laatste is het dynamische obstakel, voorlopig nog maar 1 dynamisch
                
        def ridgeback_control(self):
                 # create environment
                rospy.init_node('ridgeback_omg_control', anonymous=True) # initialize node 'ridgeback_control'
                rate = rospy.Rate(100)  # 100hz publishing rate
                
                
                #remove obstacles or load everything again, maybe do it every other iteration 
                # Obstacle
                #rectangle = Rectangle(width=3., height=0.2)
                #self.environment.add_obstacle(Obstacle({'position': [-2.1, 0.5]}, shape=rectangle))
                #self.environment.add_obstacle(Obstacle({'position': [1.7, 0.5]}, shape=rectangle))
                #self.problem.init()
                
                # simulation of a motion planning application: go through 3 via points, while
                # an obstacle is changing position
                t00 = time.time()
                state_traj = np.c_[self.current_state]
                input_traj = np.c_[[0.0, 0.0]]
 
                target_reached = False
                saved = True #dit is voor plottingpurposes
                while not rospy.is_shutdown():
                        speed = Twist()  # make Twist object speed
                        for via_point in (self.via_points):
                                self.vehicle.set_terminal_conditions(via_point)
                                
                                self.vehicle.set_initial_conditions(via_point)  # for init guess
                                self.deployer.reset()  # let's start from new initial guess
                                while not target_reached:
                                        #update time
                                        t0 = time.time() - t00
##                                      update position/ environment dit moet in tijdelijke variabelen want anders kan binnen 1 berekening de positie of environment verandere

                                        #self.environment.obstacles=[]
   #                                     print self.environment.obstacles[0]
                                        #self.environment.obstacles=self.stat_obs+self.dyn_obstacles
   #                                     print self.environment.obstacles
                                        #time.sleep(5)
                                        #update all dynamic obstacles by deleting and adding because there is no updating
        #                               DELETE ALL EXISTING DYNAMIC OBSTACLES
        #                               for OBJ in dyn_obstacles:
        #                                       trajectory = {'velocity': {'time': [time.time()], 'values': [[OBJ.x_speed,OBJ.y_speed]]}}
        #                                       simulation = {'trajectories': trajectory}
        #                                       if OBJ==circle:
        #                                               environment.add_obstacle(Obstacle({'position': [OBJ.x_pos, OBJ.y_pos]}, shape=Circle(OBJ.radius),
        #                                   simulation=simulation))
        #                                       elif OBJ==rectangle:
        #                                               environment.add_obstacle(Obstacle({'position': [OBJ.x_pos, OBJ.y_pos]}, shape=Rectangle(OBJ.width,OBJ.height,OBJ.orientation),
        #                                   simulation=simulation))
        #                                               
        #                                       
        #                               environment.obstacles.pop() #laatste is het dynamische obstakel, voorlopig nog maar 1 dynamisch
        #                               time.sleep(5)
                                        self.dyn_obstacles[0]=circle_obs(20.,20.,.1,0.,0.)
                                        if (t0 - self.current_time -self.update_time) >= 0.:
                                                self.current_time = t0
                                                #self.i=self.i+1
                                                #if self.dyn_msg==1:   
                                                              #then update dyn_obs else no message has been sent
                                                #if self.i==8:
                                                 #   for obs in self.dyn_obstacles:
                                                  #      obs.shape=Circle(10000.)
                                                        #self.environment.obstacles[0].set_state({'position': [2.0, 2.0]})
                                                #self.dyn_msg=0  #Set boolean to zero after update
                                                #print self.i             
                                                # 'measure' current state (here ideal trajectory following is simulated)
                                                #if state_traj.shape[1] > 1:
                                                        #self.current_state = state_traj[:, -self.n_samp - 1]
                                                self.current_state = np.array([self.pose_x ,self.pose_y])
                                                        
        #                                       else:
        #                                               self.current_state = [pose_x, pose_y]
                                                        #self.current_state = state_traj[:, 0]
                                                # update motion planning
                 #                               print "print"
                  #                              print self.deployer.problem.environment.obstacles
                                                trajectories = self.deployer.update(self.current_time, self.current_state)
         
                                                # store state & input trajectories -> simulation of ideal trajectory following
                                                state_traj = np.c_[state_traj, trajectories['state'][:, 1:self.n_samp + 1]]
                                                input_traj = np.c_[input_traj, trajectories['input'][:, 1:self.n_samp + 1]]
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
        #                                       print state_traj[:, -1]
        #                                       print type (state_traj[:, -1])
        #                                       print self.current_state
        #                                       print type (self.current_state)
        #                                       print via_point
                                                # check target
                                                if (np.linalg.norm(via_point - self.current_state) < 5e-2 ):#and np.linalg.norm(input_traj[:, -1]) < 5e-2): nog snelheidscheck maken
                                                        
                                                        target_reached = True
                                                        print('Target reached, close enough!')
                                                        time.sleep(5)
                                                if (self.problem.iteration > 300):
                                                        target_reached = True
                                                        print('Target reached, max iterations...')
         
                        if not saved: #dit is gewoon voor plotting purposes
                                xpos = np.array(state_traj[0,:])
                                ypos = np.array(state_traj[1,:])
                                xspeed = np.array(input_traj[0,:])
                                yspeed = np.array(input_traj[1,:])
                                np.savetxt('xpos.csv', xpos, delimiter=",")
                                np.savetxt('ypos.csv', ypos, delimiter=",")
                                np.savetxt('xspeed.csv', xspeed, delimiter=",")
                                np.savetxt('yspeed.csv', yspeed, delimiter=",")
                                saved = True
                        speed.linear.x = 0
                        speed.linear.y = 0        
                        rospy.loginfo(speed)
                        pub.publish(speed)
                        rate.sleep()      
        #       # spin() simply keeps python from exiting until this node is stopped
                rospy.spin()

if __name__ == '__main__':
        try:
            ridgeback_omg_control().ridgeback_control()
        except rospy.ROSInterruptException:
                pass

