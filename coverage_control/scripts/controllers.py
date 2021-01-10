#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

# class NaiveController:

# 	def __init__(self, uid):
# 		pass


class MAVROSController:

	def __init__(self, uid):
		self.uid = uid
		self.cntr_state = None
		self.local_pos = np.zeros(2)

		self.velocity_pub = rospy.Publisher('/uas{}/mavros/setpoint_velocity/cmd_vel_unstamped'.format(self.uid), Twist, queue_size=1)
		self.position_pub = rospy.Publisher('/uas{}/mavros/setpoint_position/local'.format(self.uid), PoseStamped, queue_size=1)

		self.local_pos_sub = rospy.Subscriber('/uas{}/mavros/global_position/local', Odometry, self.local_pos_cb, queue_size=1)
		self.state_sub = rospy.Subscriber('/uas{}/mavros/state'.format(self.uid), State, self.cntr_state_cb, queue_size=1)

		self.stream_rate_client = rospy.ServiceProxy('/uas{}/mavros/set_stream_rate'.format(self.uid), StreamRate)
		self.set_mode_client = rospy.ServiceProxy('/uas{}/mavros/set_mode'.format(self.uid), SetMode)
		self.arm_client = rospy.ServiceProxy('/uas{}/mavros/cmd/arming'.format(self.uid), CommandBool)
		self.takeoff_client = rospy.ServiceProxy('/uas{}/mavros/cmd/takeoff'.format(self.uid), CommandTOL)
		self.land_client = rospy.ServiceProxy('/uas{}/mavros/cmd/land'.format(self.uid), CommandTOL)

	def local_pos_cb(self, msg):
		self.local_pos = np.array([msg.pose.position.x, msg.pose.position.y], dtype=float)

	def cntr_state_cb(self, msg):
		self.cntr_state = msg

	def set_stream_rate(self, rate):
		self.stream_rate_client.wait_for_service()
		self.stream_rate_client(0, rate, True)
		rospy.loginfo('UAS {0} will stream data at rate {1} Hz.'.format(self.uid, rate))

	def set_mode(self, mode):
		self.set_mode_client.wait_for_service()
		res = self.set_mode_client(0, mode)
		if res.success:
			rospy.loginfo('(Success) UAS {0} mode set to {1}.'.format(self.uid, mode))
		else:
			if self.cntr_state:
				rospy.logwarn('(Fail) UAS {0} mode remains at {1}.'.format(self.uid, self.cntr_state.mode))
			else:
				rospy.logwarn('(Fail) UAS {} have not had valid feedback, yet.'.format(self.uid))

	def arm(self, value):
		self.arm_client.wait_for_service()
		res = self.arm_client(value)
		if res.success:
			rospy.loginfo('(Success) UAS {0} is {1}.'.format(self.uid, "ARMED" if value else "DISARMED"))
		else:
			rospy.logwarn('(Fail) UAS {0} remains {1}.'.format(self.uid, "DISARMED" if value else "ARMED"))

	def takeoff(self, h=10.):
		self.takeoff_client.wait_for_service()
		res = self.takeoff_client(0, 0, 0, 0, h)
		if res.success:
			rospy.loginfo('(Success) UAS {0} TAKEOFF {1}.'.format(self.uid, res.result))
		else:
			rospy.logwarn('(Fail) UAS {0} TAKEOFF {1}.'.format(self.uid, res.result))

	def land(self):
		self.land_client.wait_for_service()
		res = self.land_client(0, 0, 0, 0, 0.1)
		if res.success:
			rospy.loginfo('(Success) UAS {0} LAND {1}.'.format(self.uid, res.result))
		else:
			rospy.logwarn('(Fail) UAS {0} LAND {1}.'.format(self.uid, res.result))

	def initialize(self, stream_rate=4., height=10.):
		rospy.loginfo("UAS {} connecting...")

		# aux_rate = rospy.Rate(stream_rate)
		# while not rospy.is_shutdown() and not self.cntr_state.connected:
		# 	aux_rate.sleep()

		while not self.cntr_state.connected:
			# aux_rate.sleep()
			continue

		rospy.loginfo("UAS {} setting stream rate...")
		self.set_stream_rate(stream_rate)

		rospy.loginfo("UAS {} setting to GUIDED mode...")
		# while not rospy.is_shutdown() and not self.cntr_state.mode != "GUIDED":
		# 	self.set_mode("GUIDED")
		# 	aux_rate.sleep()

		while not self.cntr_state.mode != "GUIDED":
			self.set_mode("GUIDED")
			# aux_rate.sleep()
			# continue

		rospy.loginfo("UAS {} arming...")
		time.sleep(1.0)

		# while not rospy.is_shutdown() and not self.cntr_state.armed:
		# 	self.arm(True)
		# 	aux_rate.sleep()

		while not self.cntr_state.armed:
			self.arm(True)
			# aux_rate.sleep()

		rospy.loginfo("UAS {} trying to takeoff...")
		time.sleep(5.0)
		self.takeoff(height)

	def send_vel_cmd(self, vx, vy, vz=0.0):
		vel_cmd = Twist()
		vel_cmd.linear.x = vx
		vel_cmd.linear.y = vy
		vel_cmd.linear.z = vz
		self.velocity_pub.publish(vel_cmd)

	def send_pos_cmd(self, px, py, pz):
		pos_cmd = PoseStamped()
		pos_cmd.pose.position.x = px
		pos_cmd.pose.position.y = py
		pos_cmd.pose.position.z = pz
		self.position_pub.publish(pos_cmd)


# class AirSimSimpleFlightController:

# 	def __init__(self, uid):
# 		pass