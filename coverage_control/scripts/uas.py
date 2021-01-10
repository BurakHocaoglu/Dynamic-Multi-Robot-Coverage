#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

from cvxopt import matrix, solvers
from coverage_control.msg import FlightState, FlightStateCompressed, VCell, SensingInfo

class UAS:

	def __init__(self, uid):
		self.uid = uid
		self.grid_dims = None
		self.grid_offset = None
		self.grid_pos = None
		self.grid_resolution = 1.
		self.position = np.zeros(2)
		self.velocity = np.zeros(2)
		self.goal = np.zeros(2)
		self.voronoi_cell = []
		self.args = dict()
		self.nb_states = dict()
		self.boundary = []
		self.boundary_vertices = []
		self.logger = None
		self.converged = False
		self.setup()

		self.whole_frontier = []
		self.immediate_frontier = []

		self.H = matrix([[2., 0.], [0., 2.]], tc='d')

		self.flight_state_sub = rospy.Subscriber('/flight_states', FlightStateCompressed, self.flight_state_cb)

		self.flight_state_pub = rospy.Publisher('/flight_states', FlightStateCompressed, queue_size=1)
		self.voronoi_cell_pub = rospy.Publisher('/voronoi_cells', VCell, queue_size=1)
		self.sense_info_pub = rospy.Publisher('/sensing_info', SensingInfo, queue_size=1)

	def flight_state_cb(self, msg):
		if msg.id == self.uid:
			return

		if self.nb_states.get(msg.id) is None:
			self.nb_states[msg.id] = {'pos': np.array([msg.x, msg.y], dtype=float), 'vel': np.array([msg.vx, msg.vy], dtype=float)}

		else:
			self.nb_states[msg.id]['pos'] = np.array([msg.x, msg.y], dtype=float)
			self.nb_states[msg.id]['vel'] = np.array([msg.vx, msg.vy], dtype=float)

	def setup(self):
		try:
			init = rospy.get_param('/init{}'.format(self.uid), None)
			if init is None:
				rospy.logwarn('UAS {} could not find: init!'.format(self.uid))
				sys.exit(-1)

			else:
				self.position = np.array(init, dtype=float)
				rospy.loginfo('UAS {0} will start from position {1}.'.format(self.uid, self.position))

			_boundary = rospy.get_param('/boundary', None)
			if _boundary is None:
				rospy.logwarn('UAS {} could not find: boundary!'.format(self.uid))
				sys.exit(-1)

			else:
				x_comps, y_comps = zip(*_boundary)
				self.args['xlim'] = [min(x_comps), max(x_comps)]
				self.args['ylim'] = [min(y_comps), max(y_comps)]
				rospy.loginfo('UAS {0} has set space limits from X({1}) to Y({2}).'.format(self.uid, self.args['xlim'], self.args['ylim']))

				np_boundary = [np.array(bv, dtype=float) for bv in _boundary]
				sorted_boundary = angular_sort(np.mean(np_boundary, axis=0), np_boundary)
				self.boundary_vertices = list(sorted_boundary)
				sorted_boundary.append(sorted_boundary[0])
				for i in range(len(sorted_boundary) - 1):
					middle = (sorted_boundary[i] + sorted_boundary[i + 1]) * 0.5
					direction = sorted_boundary[i + 1] - sorted_boundary[i]
					normal = np.array([-direction[1], direction[0]], dtype=float)
					normal *= np.linalg.norm(normal)
					self.boundary.append((normal, middle))

				self.grid_resolution = rospy.get_param('/grid_res', 1.)
				W = self.args['xlim'][1] - self.args['xlim'][0]
				H = self.args['ylim'][1] - self.args['ylim'][0]
				self.grid_dims = (H, W)
				self.grid_offset = (self.args['xlim'][0], self.args['ylim'][0])
				self.grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)

				rospy.loginfo('UAS {} will start in grid position: {}.'.format(self.uid, self.grid_pos))
				rospy.loginfo('UAS {} has loaded boundary bisectors.'.format(self.uid))

			rate = rospy.get_param('/rate', None)
			if rate is None:
				rospy.logwarn('UAS {} could not find: rate! Setting to default 4 Hz.'.format(self.uid))
				self.args['rate'] = 4.

			else:
				self.args['rate'] = rate
				rospy.loginfo('UAS {0} will run at rate {1} Hz.'.format(self.uid, self.args['rate']))

			vmax = rospy.get_param('/vmax', None)
			if vmax is None:
				rospy.logwarn('UAS {} could not find: vmax! Setting to default 1 m/s.'.format(self.uid))
				self.args['vmax'] = 1.

			else:
				self.args['vmax'] = vmax
				rospy.loginfo('UAS {0} will have max velocity magnitude at {1} m/s.'.format(self.uid, self.args['vmax']))

			logdir_prefix = rospy.get_param('/logdir_prefix', None)
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
		self.sense_info_pub.publish(SensingInfo(self.position[0], self.position[1], self.grid_pos[0], self.grid_pos[1], self.uid))

	def broadcast_cell(self):
		vcell_data = []
		for v in self.voronoi_cell:
			vcell_data.extend(list(v))

		vcell_data.extend(list(self.voronoi_cell[0]))

		self.voronoi_cell_pub.publish(VCell(self.uid, 2, vcell_data))

	def dump_bisectors(self, bisectors, end=False, after=0):
		self.logger.write('UAS {} - Bisectors:\n'.format(self.uid))
		i = 0
		for b in bisectors:
			if i == after:
				self.logger.write('==================================\n')

			self.logger.write('\tBisector:\n')
			self.logger.write('\t\tNormal: {}\n'.format(b[0]))
			self.logger.write('\t\tMiddle: {}\n'.format(b[1]))
			self.logger.write('\t-\n')
			i += 1
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
		return np.all(product <= b + .5), diff

	def dump_nb_states(self):
		self.logger.write('UAS {} - Neighbour States:\n'.format(self.uid))
		for nid, nfs in self.nb_states.items():
			self.logger.write('\t- UAS {0} at {1}\n'.format(nid, nfs['pos']))
		self.logger.write('---\n')

	def compute_boundary_constraints(self):
		try:
			constraints, values = [], []

			for b in self.boundary:
				constraints.append(b[0])
				values.append((b[1].dot(b[0])).round(2))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)
			return A_cell, b_cell

		except Exception as e:
			print(traceback.format_exc())

	def compute_cell_constraints(self):
		try:
			constraints, values = [], []

			for _, fs in self.nb_states.items():
				normal = (fs['pos'] - self.position).round(2)
				middle = ((fs['pos'] + self.position) * 0.5).round(2)
				constraints.append(normal)
				values.append((middle.dot(normal)).round(2))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)
			return A_cell, b_cell

		except Exception as e:
			print(traceback.format_exc())

	def compute_bisectors(self):
		try:
			bisectors, vbisectors = [], []
			constraints, values = [], []
			tol = 0.1

			bisectors.extend(self.boundary)
			self.dump_bisectors(bisectors)

			for _, fs in self.nb_states.items():
				normal = (fs['pos'] - self.position).round(2)
				middle = ((fs['pos'] + self.position) * 0.5).round(2)
				constraints.append(normal)
				values.append((middle.dot(normal)).round(2))
				bisectors.append((normal, middle))
				vbisectors.append((normal, middle))

			# self.dump_nb_states()
			self.dump_bisectors(bisectors, end=True, after=len(self.boundary))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)
			self.voronoi_cell = []

			self.logger.write('UAS {} - Voronoi Construction:\n'.format(self.uid))
			for i in range(len(vbisectors)):
				n_i, m_i = vbisectors[i]
				d_i = m_i.dot(n_i)

				for j in range(len(bisectors)):
					n_j, m_j = bisectors[j]
					d_j = m_j.dot(n_j)

					if np.arctan2(n_i[1], n_i[0]) == np.arctan2(n_j[1], n_j[0]):
						continue

					try:
						A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
						b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
						p = np.linalg.solve(A_, b_).round(2)

					except np.linalg.LinAlgError:
						continue

					except:
						print(traceback.format_exc())
						continue

					inside_check = is_in_space(self.args['xlim'], self.args['ylim'], p, tol)
					feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)
					if inside_check and feasibility_check:
						self.voronoi_cell.append(p)
						# self.logger.write('\t- {} X {} = {} - Success!\n'.format(bisectors[i], bisectors[j], p))

					else:
						# self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(bisectors[i], bisectors[j], p, inside_check, feasibility_check))
						self.logger.write('\t- Intersection vertex {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}, diff: {}\n'.format(p, inside_check, feasibility_check, diff))

			v_eligible = []
			for v in self.boundary_vertices:
				feasibility, diff = self.check_feasibility(A_cell, b_cell, v)
				if feasibility:
					v_eligible.append(v)

				else:
					self.logger.write('\t- Boundary vertex {} - Failure! ||| INFEASIBLE, diff: {}\n'.format(v, diff))

			self.voronoi_cell = angular_sort(self.position, self.voronoi_cell)
			segment_start = get_index_of_angular_insert(self.voronoi_cell, v_eligible[0], self.position)

			if len(v_eligible) == 1:
				self.voronoi_cell.insert(segment_start, v_eligible[0])
				# self.logger.write('UAS {} inserting boundary vertex {}\n'.format(self.uid, v_eligible[0]))

			elif len(v_eligible) > 1:
				# segment_end = get_index_of_angular_insert(self.voronoi_cell, v_eligible[-1], self.position)
				# self.voronoi_cell[segment_start:segment_start + len(v_eligible)] = v_eligible
				self.voronoi_cell[segment_start:segment_start] = v_eligible
				# self.logger.write('UAS {} inserting boundary vertices {}\n'.format(self.uid, v_eligible))

			else:
				# self.logger.write('UAS {} inserting no boundary vertices.\n'.format(self.uid))
				pass

			# if not check_angle_integrity(self.voronoi_cell, self.position):
			# 	self.logger.write("************************************\n")
			# 	self.logger.write("UAS {} has no angle integrity in it's voronoi_cell:\n".format(self.uid))
			# 	for v in self.voronoi_cell:
			# 		self.logger.write("\t{}\n".format(v))
			# 	self.logger.write("************************************\n")

			A_iq = matrix(A_cell, tc='d')
			b_iq = matrix(b_cell, tc='d')
			self.dump_voronoi_cell()
			self.broadcast_cell()

			return A_iq, b_iq
		except Exception as e:
			print(traceback.format_exc())

	def set_goal(self, g):
		change = np.linalg.norm(g - self.goal)
		self.converged = change <= 0.1
		self.goal = np.array(g, dtype=float)
		# rospy.loginfo('UAS {0} goal set to {1}.'.format(self.uid, self.goal))

	def solve_step(self):
		try:
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
		except Exception as e:
			print(traceback.format_exc())

	def vel_pub(self):
		vel_cmd = Twist()
		vel_cmd.linear.x = self.velocity[0]
		vel_cmd.linear.y = self.velocity[1]
		self.velocity_pub(vel_cmd)

	def execute_step(self, v_next):
		# self.logger.write('UAS {0} - Before: {1}\n'.format(self.uid, self.position))
		# self.position = self.position + v_next / self.args['rate']
		# self.logger.write('UAS {0} - After {1}: {2}\n'.format(self.uid, v_next, self.position))
		# self.logger.write('-----\n')
		# self.velocity = v_next

		self.position = self.position + v_next / self.args['rate']
		self.grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)
		# self.position = real_to_grid(self.position + v_next / self.args['rate'], self.grid_offset, 1)
		self.velocity = v_next