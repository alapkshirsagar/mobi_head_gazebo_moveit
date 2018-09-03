#!/usr/bin/env python
import roslib
roslib.load_manifest('mobi_head')

import rospy
import actionlib
from std_msgs.msg import Float64, String
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

import time
import numpy as np

import threading

class Head:
	def __init__(self, head_name):
		self.name = head_name           
		self.jta = actionlib.SimpleActionClient('/dynamixel_controller/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')

		self.init_trajectories()

		self.breathe()
		#self.look_around()
      
      
	def init_trajectories(self):
      
		# laugh trajectory

		self.laugh_trajectory_goal = FollowJointTrajectoryGoal()
		self.laugh_trajectory_goal.trajectory.header.frame_id = 'base_link'
		self.laugh_trajectory_goal.trajectory.joint_names = ['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		n = 100
		dt = 0.025 # 0.025
		rps = 1.75 # 0.25

		for i in range (n):
			p = JointTrajectoryPoint()
			theta = rps*2.0*np.pi*i*dt
			x1 = -0.035*np.sin(1*theta) - i * 0.0035
			x2 =  0.05*np.sin(2*theta)

			p.positions.append(0)
			p.positions.append(x1)
			p.positions.append(x2)
			p.positions.append(0)

			self.laugh_trajectory_goal.trajectory.points.append(p)

			# set duration
			self.laugh_trajectory_goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)
			#rospy.loginfo("test: angles[%d][%f, %f]",n,x1,x2)

		# breathe trajectory
		self.breathe_trajectory_goal = FollowJointTrajectoryGoal()
		self.breathe_trajectory_goal.trajectory.header.frame_id = 'base_link'
		self.breathe_trajectory_goal.trajectory.joint_names = ['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		dt = 0.025 # 0.025
		rps = 0.15 # 0.25
		i = 0

		self.breathe_duration = 0

		for theta in np.arange (0, 2*np.pi, np.pi/90):
			p = JointTrajectoryPoint()
			#theta = rps*2.0*math.pi*i*dt
			x1 = 0.25*(0.5*np.cos(theta) - 0.5)
			x2 = 0.15*(0.5*np.cos(theta) - 0.5)

			p.positions.append(0)
			p.positions.append(x1)
			p.positions.append(x2)
			p.positions.append(0)

			self.breathe_trajectory_goal.trajectory.points.append(p)

			# set duration
			self.breathe_trajectory_goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)

			i += 1

		self.breathe_duration = i * dt

		# look around trajectory
		self.look_around_trajectory_goal = FollowJointTrajectoryGoal()
		self.look_around_trajectory_goal.trajectory.header.frame_id = 'base_link'
		self.look_around_trajectory_goal.trajectory.joint_names = ['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		i = 0

		for theta in np.arange (0, np.pi, np.pi/180):
			p = JointTrajectoryPoint()
			#theta = rps*2.0*math.pi*i*dt
			x1 = -0.5*np.sin(2*theta)
			x2 =  -0.1*np.sin(theta)

			p.positions.append(x1)
			p.positions.append(x2)
			p.positions.append(x2)
			p.positions.append(x1/2)

			self.look_around_trajectory_goal.trajectory.points.append(p)

			# set duration
			self.look_around_trajectory_goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)

			i += 1

		# yes trajectory
		self.yes_trajectory_goal = FollowJointTrajectoryGoal()
		self.yes_trajectory_goal.trajectory.header.frame_id = 'base_link'
		self.yes_trajectory_goal.trajectory.joint_names = ['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		dt = 0.025 # 0.025
		rps = 1.75 # 0.25

		i = 0

		for theta in np.arange (0, 2*np.pi, np.pi/45):
			p = JointTrajectoryPoint()
			x1 = -0.35*np.sin(theta)
			#x2 =  0.05*np.sin(2*theta)

			p.positions.append(0)
			p.positions.append(0)
			p.positions.append(x1)
			p.positions.append(0)

			self.yes_trajectory_goal.trajectory.points.append(p)

			# set duration
			self.yes_trajectory_goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)

			i += 1
      
	  
	def execute_trajectory(self, traj):
		traj.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(2)

		#rospy.loginfo("traj: ", traj)
		self.jta.send_goal_and_wait(traj)
      
	def move_head_demo(self):
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(2)
		goal.trajectory.header.frame_id = 'base_link'
		goal.trajectory.joint_names = ['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		n = 400
		dt = 0.025 # 0.025
		rps = 0.15 # 0.25

		for i in range (n):
			p = JointTrajectoryPoint()
			theta = rps*2.0*math.pi*i*dt
			x1 = -0.5*math.sin(2*theta)
			x2 =  0.1*math.sin(1*theta)

			p.positions.append(x1)
			p.positions.append(x2)
			#p.positions.append(x1)
			p.positions.append(x2)
			p.positions.append(x2)

			goal.trajectory.points.append(p)

			# set duration
			goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)
			#rospy.loginfo("test: angles[%d][%f, %f]",n,x1,x2)

		self.jta.send_goal_and_wait(goal)
      
      
	def command_callback(self, data):
		print 'command received: ', data

		if (data.data == 'neutral'):
			#self.move_head_demo()
			self.execute_trajectory(self.breathe_trajectory_goal)

		elif (data.data == 'surprised'):
			# execute surprised trajectory
			pass
		
		elif (data.data == 'happy'):
			# execute happy trajectory
			pass

		elif (data.data == 'sad'):
			# execute sad trajectory
			pass

		elif (data.data == 'angry'):
			# execute angry trajectory
			pass

		elif (data.data == 'inlove'):
			# execute inlove trajectory
			pass

		elif (data.data == 'shy'):
			# execute shy trajectory
			pass

		elif (data.data == 'yes'):
			# execute yes trajectory
			self.execute_trajectory(self.yes_trajectory_goal)

		elif (data.data == 'no'):
			# execute no trajectory
			pass

		elif (data.data == 'ok'):
			# execute ok trajectory
			pass

		elif (data.data == 'sorry'):
			# execute sorry trajectory
			pass
	
	def breathe(self):
		print 'breathing...'
		self.execute_trajectory(self.breathe_trajectory_goal)
		threading.Timer(self.breathe_duration - 0.5, self.breathe).start()
      
	def look_around(self):
		print 'looking around'
		self.execute_trajectory(self.look_around_trajectory_goal)
		threading.Timer(np.random.randint(20, 40, size = 1), self.look_around).start()

	def be_sad(self):
		print 'being sad'
		self.execute_trajectory(self.sad_trajectory_goal)

def main():
	head = Head('head')

	rospy.loginfo('head initialized')

	rospy.Subscriber("/mobi_head_command", String, head.command_callback)

	rospy.spin()
  
                        
if __name__ == '__main__':
	rospy.init_node('mobi_head_trajectory_controller')

	main()