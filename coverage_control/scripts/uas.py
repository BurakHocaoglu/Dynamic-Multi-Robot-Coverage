#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

from cvxopt import matrix, solvers
from coverage_control.msg import FlightState, FlightStateCompressed, VCell

class UAS:

	def __init__(self, uid, paramFile):
		self.uid = uid
		self.position = np.zeros(2)
		self.velocity = np.zeros(2)
		self.goal = np.zeros(2)
		self.voronoi_cell = []
		self.args = dict()
		self.nb_states = dict()
		self.boundary = []
		self.logger = None
		self.converged = False
		self.setup(paramFile)

		self.H = matrix([[2., 0.], [0., 2.]], tc='d')

		self.flight_state_sub = rospy.Subscriber('/flight_states', FlightStateCompressed, self.flight_state_cb)

		self.flight_state_pub = rospy.Publisher('/flight_states', FlightStateCompressed, queue_size=1)
		self.voronoi_cell_pub = rospy.Publisher('/voronoi_cells', VCell, queue_size=1)

	def flight_state_cb(self, msg):
		if msg.id == self.uid:
			return

		if self.nb_states.get(msg.id) is None:
			self.nb_states[msg.id] = {'pos': np.array([msg.x, msg.y], dtype=float), 'vel': np.array([msg.vx, msg.vy], dtype=float)}

		else:
			self.nb_states[msg.id]['pos'] = np.array([msg.x, msg.y], dtype=float)
			self.nb_states[msg.id]['vel'] = np.array([msg.vx, msg.vy], dtype=float)

	def setup(self, paramFile):
		try:
			if paramFile is None:
				rospy.logerr()
				sys.exit(-1)

			params = None
			with open(paramFile, 'r') as PF:
				params = yaml.load(PF)

			if not params:
				rospy.logerr('Empty parameter structure!')
				sys.exit(-1)

			init = params.get('init')
			if init is None:
				rospy.logwarn('UAS {} could not find: init!'.format(self.uid))
				sys.exit(-1)

			else:
				self.position = np.array(init, dtype=float)
				rospy.loginfo('UAS {0} will start from position {1}.'.format(self.uid, self.position))

			_boundary = params.get('boundary')
			if _boundary is None:
				rospy.logwarn('UAS {} could not find: boundary!'.format(self.uid))
				sys.exit(-1)

			else:
				x_comps = map(lambda a: a[0], _boundary)
				y_comps = map(lambda a: a[1], _boundary)
				self.args['xlim'] = [min(x_comps), max(x_comps)]
				self.args['ylim'] = [min(y_comps), max(y_comps)]
				rospy.loginfo('UAS {0} has set space limits from X({1}) to Y({2}).'.format(self.uid, self.args['xlim'], self.args['ylim']))

				np_boundary = [np.array(bv, dtype=float) for bv in _boundary]
				sorted_boundary = angular_sort(np.mean(np_boundary, axis=0), np_boundary)
				sorted_boundary.append(sorted_boundary[0])
				for i in range(len(sorted_boundary) - 1):
					middle = (sorted_boundary[i] + sorted_boundary[i + 1]) * 0.5
					direction = sorted_boundary[i + 1] - sorted_boundary[i]
					normal = np.array([direction[1], -direction[0]], dtype=float)
					normal *= np.linalg.norm(normal)
					self.boundary.append((normal, middle))

				rospy.loginfo('UAS {} has loaded boundary bisectors.'.format(self.uid))

			rate = params.get('rate')
			if rate is None:
				rospy.logwarn('UAS {} could not find: rate! Setting to default 4 Hz.'.format(self.uid))
				self.args['rate'] = 4.

			else:
				self.args['rate'] = rate
				rospy.loginfo('UAS {0} will run at rate {1} Hz.'.format(self.uid, self.args['rate']))

			vmax = params.get('vmax')
			if vmax is None:
				rospy.logwarn('UAS {} could not find: vmax! Setting to default 1 m/s.'.format(self.uid))
				self.args['vmax'] = 1.

			else:
				self.args['vmax'] = vmax
				rospy.loginfo('UAS {0} will have max velocity magnitude at {1} m/s.'.format(self.uid, self.args['vmax']))

			logdir_prefix = params.get('logdir_prefix')
			if logdir_prefix is None:
				cwd = os.getcwd()
				rospy.logwarn('UAS {0} could not find: logdir_prefix! Setting to default {1}.'.format(self.uid, cwd))
				logdir_prefix = cwd

			else:
				self.args['logdir_prefix'] = logdir_prefix
				rospy.loginfo('UAS {0} will log to the path {1}.'.format(self.uid, self.args['logdir_prefix']))

			stamp = datetime.datetime.now()
			logdir = '{0}{1}'.format(self.args['logdir_prefix'], stamp.strftime('%b_%d_%Y_%H_%M'))

			if not os.path.exists(logdir):
				rospy.logwarn("Path {} does not exist. Creating...".format(logdir))
				os.makedirs(logdir)

			self.logger = open("{0}/uas{1}_log.txt".format(logdir, self.uid), 'w+')

		except Exception as e:
			print(traceback.format_exc())

		finally:
			rospy.loginfo('UAS {} has been setup.'.format(self.uid))

	def broadcast_state(self):
		self.flight_state_pub.publish(FlightStateCompressed(self.uid, self.position[0], self.position[1], self.velocity[0], self.velocity[1]))

	def broadcast_cell(self):
		vcell_data = []
		for v in self.voronoi_cell:
			vcell_data.extend(list(v))

		self.voronoi_cell_pub.publish(VCell(self.uid, 2, vcell_data))

	def dump_bisectors(self, bisectors, end=False):
		self.logger.write('UAS {} - Bisectors:\n'.format(self.uid))
		for b in bisectors:
			self.logger.write('\tBisector:\n')
			self.logger.write('\t\tNormal: {}\n'.format(b[0]))
			self.logger.write('\t\tMiddle: {}\n'.format(b[1]))
			self.logger.write('\t-\n')
		self.logger.write('---\n')

		if end:
			self.logger.write('#########################\n')

	def dump_voronoi_cell(self):
		self.logger.write('UAS {} - Voronoi Cell:\n'.format(self.uid))
		for v in self.voronoi_cell:
			self.logger.write('\t{}\n'.format(v))
		self.logger.write('-----\n')

	def check_feasibility(self, A, b, p):
		product = A.dot(p)
		diff = product - b
		# self.logger.write('\t\tUAS {0} feasibility check of A:{1} p:{2} b:{3} has diff: {4}\n\n'.format(self.uid, A, p, b, diff))
		return np.all(product <= b + .1)

	def dump_nb_states(self):
		self.logger.write('UAS {} - Neighbour States:\n'.format(self.uid))
		for nid, nfs in self.nb_states.items():
			self.logger.write('\t- UAS {0} at {1}\n'.format(nid, nfs['pos']))
		self.logger.write('---\n')

	def compute_bisectors(self):
		bisectors = []
		constraints, values = [], []
		tol = 0.1

		bisectors.extend(self.boundary)
		self.dump_bisectors(bisectors)

		for _, fs in self.nb_states.items():
			normal = (fs['pos'] - self.position).round(4)
			middle = ((fs['pos'] + self.position) * 0.5).round(4)
			constraints.append(normal)
			values.append((middle.dot(normal)).round(4))
			bisectors.append((normal, middle))

		# self.dump_nb_states()
		# self.dump_bisectors(bisectors, end=True)

		A = np.array(constraints, dtype=float)
		b = np.array(values, dtype=float)
		self.voronoi_cell = []
		# self.logger.write('UAS {} - Voronoi Construction:\n'.format(self.uid))
		for i in range(len(bisectors) - 1):
			n_i, m_i = bisectors[i]
			d_i = m_i.dot(n_i)

			for j in range(i + 1, len(bisectors)):
				n_j, m_j = bisectors[j]
				d_j = m_j.dot(n_j)

				try:
					A_ = np.array([n_i.round(4), n_j.round(4)], dtype=float)
					b_ = np.array([d_i.round(4), d_j.round(4)], dtype=float)
					p = np.linalg.solve(A_, b_).round(4)

				except np.linalg.LinAlgError:
					continue

				except:
					print(traceback.format_exc())
					continue

				inside_check = is_in_space(self.args['xlim'], self.args['ylim'], p, tol)
				# feasibility_check = np.all(A.dot(p) <= b + 1.)
				feasibility_check = self.check_feasibility(A, b, p)
				if inside_check and feasibility_check:
					self.voronoi_cell.append(p)
					# self.logger.write('\t- Candidate {} - Success!\n'.format(p))

				else:
					pass
					# self.logger.write('\t- Candidate {0} - In boundary: {1}, Feasible: {2} - Fail!\n'.format(p, str(inside_check), str(feasibility_check)))

		A_iq = matrix(A, tc='d')
		b_iq = matrix(b, tc='d')
		# self.voronoi_cell = angular_sort(self.position, self.voronoi_cell)
		self.voronoi_cell = angular_sort(np.mean(self.voronoi_cell, axis=0), self.voronoi_cell)
		self.dump_voronoi_cell()
		self.broadcast_cell()

		return A_iq, b_iq

	def set_goal(self, g):
		change = np.linalg.norm(g - self.goal)
		self.converged = change <= 0.1
		self.goal = np.array(g, dtype=float)
		# rospy.loginfo('UAS {0} goal set to {1}.'.format(self.uid, self.goal))

	def solve_step(self):
		v_next = self.velocity
		# self.logger.write('UAS {0} - v_next at solve_step() start: {1}\n'.format(self.uid, v_next))

		if len(self.voronoi_cell) > 3:
			self.voronoi_cell.append(self.voronoi_cell[0])
			self.set_goal(get_centroid(self.voronoi_cell, self.uid))
			v_next = self.goal - self.position

			# self.logger.write('UAS {0} - voronoi cell has >= 3 vertices -> v_next: {1}\n'.format(self.uid, v_next))
			_norm = np.linalg.norm(v_next)

			if _norm > self.args['vmax']:
				v_next *= self.args['vmax'] / _norm
				# self.logger.write('UAS {0} - normalized v_next: {1}\n'.format(self.uid, v_next))

			# if np.any(np.isnan(v_next)):
			# 	v_next = np.zeros(2)

			return v_next

		# self.logger.write('UAS {0} - voronoi cell has < 3 vertices -> v_next: {1}\n'.format(self.uid, v_next))
		return np.zeros(2)

	def vel_pub(self):
		vel_cmd =  Twist()
		vel_cmd.linear.x = self.velocity[0]
		vel_cmd.linear.y = self.velocity[1]
		self.velocity_pub(vel_cmd)

	def execute_step(self, v_next):
		# self.logger.write('UAS {0} - Before: {1}\n'.format(self.uid, self.position))
		self.position = self.position + v_next / self.args['rate']
		# self.logger.write('UAS {0} - After {1}: {2}\n'.format(self.uid, v_next, self.position))
		# self.logger.write('-----\n')
		self.velocity = v_next