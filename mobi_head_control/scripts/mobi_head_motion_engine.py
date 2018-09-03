#!/usr/bin/env python

__author__ = "Tekin Mericli"
__copyright__ = "Copyright (C) 2015 Tekin Mericli"
__license__ = "GPL"
__version__ = "1.0"

import roslib
roslib.load_manifest('mobi_head')

import rospy
import actionlib
import time
import numpy as np
import threading

# messages
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
# services
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed


class MobiHeadMotionEngine:

	def __init__(self, head_name):

		self.name = head_name           
		self.jta = actionlib.SimpleActionClient('/dynamixel_controller/' + self.name + '_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
		rospy.loginfo('Waiting for joint trajectory action')
		self.jta.wait_for_server()
		rospy.loginfo('Found joint trajectory action!')

		self.init_trajectories()

		#self.breathe()
		#self.look_around()
		
	def init_trajectories(self):

		# final poses for different emotions
		self.neutral_pose = {'Neck_Pan_Joint' : 0.0, 'Neck_Tilt_Joint' : 0.0, 'Head_Tilt_Joint' : 0.0, 'Head_Roll_Joint' : 0.0}
		self.happy_pose = {'Neck_Pan_Joint' : 0.0, 'Neck_Tilt_Joint' : -0.44, 'Head_Tilt_Joint' : -0.4, 'Head_Roll_Joint' : -0.18}
		self.sad_pose = {'Neck_Pan_Joint' : -0.2, 'Neck_Tilt_Joint' : 0.2, 'Head_Tilt_Joint' : -0.25, 'Head_Roll_Joint' : 0.14}
		self.angry_pose = {'Neck_Pan_Joint' : 0.0, 'Neck_Tilt_Joint' : 0.015, 'Head_Tilt_Joint' : -0.3, 'Head_Roll_Joint' : 0.0}
		self.surprised_pose = {'Neck_Pan_Joint' : 0.0, 'Neck_Tilt_Joint' : 0.26, 'Head_Tilt_Joint' : 0.43, 'Head_Roll_Joint' : 0.025}
		self.shy_pose = {'Neck_Pan_Joint' : 0.01, 'Neck_Tilt_Joint' : -0.36, 'Head_Tilt_Joint' : -0.6, 'Head_Roll_Joint' : -0.19}
		self.inlove_pose = {'Neck_Pan_Joint' : 0.0, 'Neck_Tilt_Joint' : 0.22, 'Head_Tilt_Joint' : 0.34, 'Head_Roll_Joint' : -0.14}

		# looking around the left side of the room
		self.gaze_target_1 = {'Neck_Pan_Joint' : 0.8, 'Neck_Tilt_Joint' : -0.44, 'Head_Tilt_Joint' : 0.3, 'Head_Roll_Joint' : 0.1}
		self.gaze_target_2 = {'Neck_Pan_Joint' : 0.55, 'Neck_Tilt_Joint' : 0.2, 'Head_Tilt_Joint' : 0.3, 'Head_Roll_Joint' : -0.1}
		self.gaze_target_3 = {'Neck_Pan_Joint' : 0.2, 'Neck_Tilt_Joint' : 0.0, 'Head_Tilt_Joint' : 0.1, 'Head_Roll_Joint' : 0.0}

		# looking around the right side of the room
		self.gaze_target_4 = {'Neck_Pan_Joint' : -0.2, 'Neck_Tilt_Joint' : 0.0, 'Head_Tilt_Joint' : -0.1, 'Head_Roll_Joint' : 0.0}
		self.gaze_target_5 = {'Neck_Pan_Joint' : -0.55, 'Neck_Tilt_Joint' : -0.2, 'Head_Tilt_Joint' : -0.3, 'Head_Roll_Joint' : 0.1}
		self.gaze_target_6 = {'Neck_Pan_Joint' : -0.8, 'Neck_Tilt_Joint' : 0.44, 'Head_Tilt_Joint' : -0.3, 'Head_Roll_Joint' : -0.1}

		#self.gaze_target_7 = {'Neck_Pan_Joint' : -0.50, 'Neck_Tilt_Joint' : 0.0, 'Head_Tilt_Joint' : 0.0, 'Head_Roll_Joint' : -0.1}
		#self.gaze_target_8 = {'Neck_Pan_Joint' : -0.75, 'Neck_Tilt_Joint' : 0.0, 'Head_Tilt_Joint' : 0.0, 'Head_Roll_Joint' : 0.1}



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

		# no trajectory
		self.no_trajectory_goal = FollowJointTrajectoryGoal()
		self.no_trajectory_goal.trajectory.header.frame_id = 'base_link'
		self.no_trajectory_goal.trajectory.joint_names = ['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		dt = 0.025 # 0.025
		rps = 1.75 # 0.25

		i = 0

		for theta in np.arange (0, 2*np.pi, np.pi/45):
			p = JointTrajectoryPoint()
			x1 = -0.35*np.sin(theta)
			#x2 =  0.05*np.sin(2*theta)

			p.positions.append(x1)
			p.positions.append(0)
			p.positions.append(0)
			p.positions.append(0)

			self.no_trajectory_goal.trajectory.points.append(p)

			# set duration
			self.no_trajectory_goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)

			i += 1

	""" ---------------------------TEST OF NEW trajectory_to_pose"----------------------

	def trajectory_to_pose(self, pose, velocity):

		print 'num_trajectory_points: ', num_trajectory_points, ' dt: ', dt

		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(2)
		goal.trajectory.header.frame_id = 'base_link'
		goal.trajectory.joint_names = self.joint_states.name#['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		# get the current joint values
		current_joint_values = np.array(self.joint_states.position)
		# get the goal joint values
		goal_joint_values = np.asarray([pose[j] for j in self.joint_states.name])

		trajectory_joint_values = np.array([np.linspace(s, g, num_trajectory_points) for s, g in zip(current_joint_values, goal_joint_values)])

		trajectory_distances = np.array()

		for i in range(goal_joint_values):
			dist = goal_joint_values[i]-current_joint_values[i]
			duration = dist/velocity

			num_trajectory_points = int(duration * 50)
			dt = duration / num_trajectory_points #duration per trajectory point
			#duration for each joint

		for val in range(num_trajectory_points): 
			dist = current_joint_values[val]-goal_joint_values[val]
			print dist
			dt = dist/velocity

		#print trajectory_joint_values

		#for i in range(num_trajectory_points):
			p = JointTrajectoryPoint()
			
			#for j in range(len(self.joint_states.name)):
				#p.positions.append(trajectory_joint_values[j,i])
			
			#goal.trajectory.points.append(p)

			#print 'p: ', p



			# set duration
			goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)


		return goal"""

	"""-------------------------------OLD trajectory_to_pose----------------------------"""

	# construct a trajectory through linear interpolation from the current pose of the robot to the desired pose
	def trajectory_to_pose(self, pose, duration):

		num_trajectory_points = int(duration * 50)
		dt = duration / num_trajectory_points #duration per trajectory point

		print 'num_trajectory_points: ', num_trajectory_points, ' dt: ', dt

		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(2)
		goal.trajectory.header.frame_id = 'base_link'
		goal.trajectory.joint_names = self.joint_states.name#['Neck_Pan_Joint', 'Neck_Tilt_Joint', 'Head_Tilt_Joint', 'Head_Roll_Joint']

		# get the current joint values
		current_joint_values = np.array(self.joint_states.position)
		# get the goal joint values
		goal_joint_values = np.asarray([pose[j] for j in self.joint_states.name])

		trajectory_joint_values = np.array([np.linspace(s, g, num_trajectory_points) for s, g in zip(current_joint_values, goal_joint_values)])

		for i in range(num_trajectory_points):
			p = JointTrajectoryPoint()
			
			for j in range(len(self.joint_states.name)):
				p.positions.append(trajectory_joint_values[j,i])
			
			goal.trajectory.points.append(p)

			#print 'p: ', p



			# set duration
			goal.trajectory.points[i].time_from_start = rospy.Duration.from_sec(i*dt)


		return goal
		

	def execute_trajectory(self, traj):

		traj.trajectory.header.stamp = rospy.Time.now()# + rospy.Duration(2)

		#rospy.loginfo("traj: ", traj)
		self.jta.send_goal_and_wait(traj)


	# moves the all of the head joints along sinusoidal trajectories for smooth trajectory execution demonstration purposes
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


	# callback function for the commands received from the OCU
	def command_callback(self, data):

		print 'command received: ', data

		if (data.data == 'neutral'):
			self.execute_trajectory(self.trajectory_to_pose(self.neutral_pose, 0.5))

		elif (data.data == 'surprised'):
			# execute surprised trajectory
			self.execute_trajectory(self.trajectory_to_pose(self.surprised_pose, 0.45))
		
		elif (data.data == 'happy'):
			# execute happy trajectory
			self.execute_trajectory(self.trajectory_to_pose(self.happy_pose, 1.0))

		elif (data.data == 'sad'):
			# execute sad trajectory
			self.execute_trajectory(self.trajectory_to_pose(self.sad_pose, 0.75))

		elif (data.data == 'angry'):
			# execute angry trajectory
			self.execute_trajectory(self.trajectory_to_pose(self.angry_pose, 0.75))

		elif (data.data == 'inlove'):
			# execute inlove trajectory
			self.execute_trajectory(self.trajectory_to_pose(self.inlove_pose, 0.5))

		elif (data.data == 'shy'):
			# execute shy trajectory
			self.execute_trajectory(self.trajectory_to_pose(self.shy_pose, 0.75))

		elif (data.data == 'yes'):
			# execute yes trajectory
			self.execute_trajectory(self.yes_trajectory_goal)

		elif (data.data == 'no'):
			# execute no trajectory
			self.execute_trajectory(self.no_trajectory_goal)

		elif (data.data == 'ok'):
			# execute ok trajectory
			pass

		elif (data.data == 'sorry'):
			# execute sorry trajectory
			pass

		elif (data.data == 'gazetarget1'):
			# execute trajectory to gaze target 1
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_1, 1.0))

		elif (data.data == 'gazetarget2'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_2, 1.0))

		elif (data.data == 'gazetarget3'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_3, 1.0))

		elif (data.data == 'gazetarget4'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_4, 1.0))

		elif (data.data == 'gazetarget5'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_5, 1.0))

		elif (data.data == 'gazetarget6'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_6, 1.0))

		elif (data.data == 'gazetarget7'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_7, 1.0))

		elif (data.data == 'gazetarget8'):
			# execute trajectory to gaze target 2
			self.execute_trajectory(self.trajectory_to_pose(self.gaze_target_8, 1.0))

	# callback function for the joint states
	def joint_state_callback(self, data):

		self.joint_states = data

		#print 'self.joint_states: \n', self.joint_states
	
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

	mhme = MobiHeadMotionEngine('head')

	rospy.loginfo('head initialized')

	# subscribe to the commands from the OCU
	rospy.Subscriber("/mobi_head_command", String, mhme.command_callback)

	# subscribe to the joint states
	rospy.Subscriber("/joint_states", JointState, mhme.joint_state_callback)	

	rospy.spin()
  
                        
if __name__ == '__main__':

	rospy.init_node('mobi_head_motion_engine')

	main()