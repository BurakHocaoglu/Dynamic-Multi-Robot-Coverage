import sys
import rospy
import threading
import traceback
import numpy as np

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, SetMode, CommandTOL, CommandBool
from geometry_msgs.msg import Twist, Pose
from coverage_control.msg import FlightState

from vgraph import Node, VGraph

class UAS:

	def __init__(self, name, init):
		self.name = name
		self.p0 = np.array(init, dtype=float)
		self.ready = False
		self.mavros_state = None
		self.flight_state = FlightState(self.name, Pose())
		self.local_pos = Pose()
		self.velocity = Twist()
		self.flight_pos = np.array(init, dtype=float)
		self.vcell = VGraph()
		self.goal = np.zeros(2)
		self.nb_states = dict()

		self.stream_service = "/{}/mavros/set_stream_rate".format(name)
		self.arm_service = "/{}/mavros/cmd/arming".format(name)
		self.takeoff_service = "/{}/mavros/cmd/takeoff".format(name)
		self.mode_service = "/{}/mavros/set_mode".format(name)
		self.land_service = "/{}/mavros/cmd/land".format(name)

		self.stream_client = rospy.ServiceProxy(self.stream_service, StreamRate)
		self.arm_client = rospy.ServiceProxy(self.arm_service, CommandBool)
		self.takeoff_client = rospy.ServiceProxy(self.takeoff_service, CommandTOL)
		self.mode_client = rospy.ServiceProxy(self.mode_service, SetMode)
		self.land_client = rospy.ServiceProxy(self.land_service, CommandTOL)

		self.mavros_state_sub = rospy.Subscriber('/{}/mavros/state'.format(self.name), State, self.mavros_state_cb)
		self.local_pos_sub = rospy.Subscriber('/{}/mavros/global_position/local'.format(self.name), Odometry, self.local_pos_cb)
		self.command_line_sub = rospy.Subscriber('/{}/command_line'.format(self.name), String, self.command_line_cb)
		self.flight_state_sub = rospy.Subscriber('/flight_state', FlightState, self.flight_state_cb)

		self.flight_state_pub = rospy.Publisher('/flight_state', FlightState, queue_size=1)
		self.velocity_pub = rospy.Publisher("/{}/mavros/setpoint_velocity/cmd_vel_unstamped".format(name), Twist, queue_size=1)

		self.valid = False
		self.terminate = False
		self.flight_thread = threading.Thread(name='{}_flight'.format(self.name), target=self.flight)

	def mavros_state_cb(self, msg):
		self.mavros_state = msg

	def local_pos_cb(self, msg):
		self.local_pos = msg.pose.pose

	def command_line_cb(self, msg):
		self.apply(msg.data)

	def flight_state_cb(self, msg):
		p = msg.pose
		self.nb_states[msg.name] = np.array([p.position.x, p.position.y], dtype=float)

	def apply(self, cmd):
		if cmd == 'guided':
			self.set_mode('GUIDED')

		elif cmd == 'arm':
			self.arm()

		elif cmd == 'disarm':
			self.disarm()

		elif cmd == 'takeoff':
			self.takeoff()

		elif cmd == 'land':
			self.land()

		elif cmd == 'move':
			self.flight_thread.start()

	def arm(self):
		rospy.wait_for_service(self.arm_service)
		self.arm_client(True)

	def disarm(self):
		rospy.wait_for_service(self.arm_service)
		self.arm_client(False)

	def set_mode(self, mode):
		rospy.wait_for_service(self.mode_service)
		self.mode_client(0, mode)

	def takeoff(self, h=10.):
		rospy.wait_for_service(self.takeoff_service)
		res = self.takeoff_client(0, 0, 0, 0, h)
		if res.success:
			self.ready = True

	def land(self):
		rospy.wait_for_service(self.land_service)
		self.land_client(0, 0, 0, 0, 0.1)

	def hasReachedGoal(self):
		p = np.array([self.flight_state.pose.position.x, self.flight_state.pose.position.y], dtype=float)
		return self.valid and np.linalg.norm(self.goal - p) <= 0.1

	def broadcastState(self):
		p = Pose()
		p.position.x = self.local_pos.position.x + self.p0[0]
		p.position.y = self.local_pos.position.y + self.p0[1]

		self.flight_state = FlightState(self.name, p)
		self.flight_state_pub.publish(self.flight_state)

	def computeBisectors(self):
		cons, vals, bisectors = [], [], []
		p_i = np.array([self.flight_state.pose.position.x, self.flight_state.pose.position.y], dtype=float)

		for nb, st in self.nb_states.items():
			p_j = np.array([st.pose.position.x, st.pose.position.y], dtype=float)
			normal = p_j - p_i
			mid = (p_i + p_j) * 0.5

			cons.append(normal)
			vals.append(normal.dot(mid))
			bisectors.append((normal, mid))

		A = np.array(cons, dtype=float)
		b = np.array(vals, dtype=float)
		self.vcell = VGraph()

		for i in range(len(bisectors)):
			n_i, m_i = bisectors[i]
			c_i = n_i.dot(m_i)

			for j in range(i + 1, len(bisectors)):
				n_i, m_i = bisectors[i]
				c_j = n_j.dot(m_j)

				try:
					A_ = np.array([n_i, n_j], dtype=float)
					b_ = np.array([c_i, c_j], dtype=float)
					p_ = np.linalg.solve(A_, b_)

				except np.linalg.LinAlgException:
					continue

				except:
					print traceback.format_exc()

				if np.all(A.dot(p_) <= b):
				#if np.linalg.norm(A.dot(p_) - b) <= 0.1:
					bisector_node_i = Node('B{}'.format(i), 'Bisector', m_i, n_i)
					bisector_node_j = Node('B{}'.format(j), 'Bisector', m_j, n_j)
					point_node_ij = Node('P{}{}'.format(i, j), 'Point', m_i)
					self.vcell.addVertexDirect(bisector_node_i)
					self.vcell.addVertexDirect(bisector_node_j)
					self.vcell.addVertexDirect(point_node_ij)
					self.addEdge(bisector_node_i, point_node_ij)
					self.addEdge(point_node_ij, bisector_node_j)

	def computeNextGoal(self):
		traversal = self.vcell.traversal()

		if len(traversal) < 3:
			self.valid = False
			return

		self.goal = np.mean(np.array(traversal, dtype=float), axis=0)

	def vel_step(self):
		v = Twist()
		p_ = np.array([self.flight_state.pose.position.x, self.flight_state.pose.position.y], dtype=float)
		v_ = self.goal - p_
		v_ /= np.linalg.norm(v_)
		v.linear.x = v_[0]
		v.linear.y = v_[1]
		self.velocity_pub.publish(v)
