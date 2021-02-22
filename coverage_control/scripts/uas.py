#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

from cvxopt import matrix, solvers
from coverage_control.msg import FlightState, FlightStateCompressed, VCell, SensingInfo

from coverage_control.srv import SetInitPos

class UAS:

	def __init__(self, uid):
		self.uid = uid
		self.grid_dims = None
		self.grid_offset = None
		self.grid_pos = None
		self.grid_resolution = 1.
		self.position = np.zeros(2)
		self.velocity = np.zeros(2)
		self.heading = 0.
		self.prev_goal = np.zeros(2)
		self.goal = np.zeros(2)
		self.voronoi_cell = []
		self.args = dict()
		self.nb_states = dict()
		self.boundary = []
		self.holes = dict()
		self.hole_segments = []
		self.boundary_vertices = []
		self.boundary_polygon = None
		self.logger = None
		self.local_grid_map = None
		self.footprint_size = 1

		self.coverage_state = 0
		self.converged_count = 0
		self.converged = False

		self.setup()

		self.frontier = dict()
		# self.immediate_frontier = []

		self.H = matrix([[2., 0.], [0., 2.]], tc='d')

		self.flight_state_sub = rospy.Subscriber('/flight_states', FlightStateCompressed, self.flight_state_cb)

		self.flight_state_pub = rospy.Publisher('/flight_states', FlightStateCompressed, queue_size=1)
		self.voronoi_cell_pub = rospy.Publisher('/voronoi_cells', VCell, queue_size=1)
		self.sense_info_pub = rospy.Publisher('/sensing_info', SensingInfo, queue_size=1)

		self.set_init_pos_srv = rospy.Service('/uas{}/set_init_pos'.format(self.uid), SetInitPos, self.handle_set_init_pos)

	def handle_set_init_pos(self, req):
		try:
			self.position = np.array([req.x, req.y], dtype=float)
			return True
		except Exception as e:
			print(traceback.format_exc())
			return False

	def flight_state_cb(self, msg):
		if msg.id == self.uid:
			return

		if self.nb_states.get(msg.id) is None:
			self.nb_states[msg.id] = {'pos': np.array([msg.x, msg.y], dtype=float), 
									  'vel': np.array([msg.vx, msg.vy], dtype=float), 
									  'end': msg.converged}

		else:
			self.nb_states[msg.id]['pos'] = np.array([msg.x, msg.y], dtype=float)
			self.nb_states[msg.id]['vel'] = np.array([msg.vx, msg.vy], dtype=float)
			self.nb_states[msg.id]['end'] = msg.converged

		if self.coverage_state == 2:
			nb_grid_pos = real_to_grid((msg.x, msg.y), self.grid_offset, self.grid_resolution)
			if self.local_grid_map is not None:
				self.local_grid_map[nb_grid_pos] = msg.id

	def all_converged(self):
		for _, st in self.nb_states.items():
			if not st['end']:
				return False

		return self.converged

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

				self.boundary_vertices = _boundary
				sorted_boundary = np.array(_boundary, dtype=float)
				for i in range(len(sorted_boundary)):
					j = (i + 1) % len(sorted_boundary)

					middle = (sorted_boundary[j] + sorted_boundary[i]) * 0.5
					direction = sorted_boundary[j] - sorted_boundary[i]

					normal = np.array([-direction[1], direction[0]], dtype=float)
					normal /= np.linalg.norm(normal)
					self.boundary.append((normal, middle, "B_{}{}".format(i, j)))

				self.grid_resolution = 1. / rospy.get_param('/grid_res', 1.)
				W = self.args['xlim'][1] - self.args['xlim'][0]
				H = self.args['ylim'][1] - self.args['ylim'][0]
				self.grid_dims = (int(H), int(W))
				self.local_grid_map = - np.ones(self.grid_dims)
				self.grid_offset = (self.args['xlim'][0], self.args['ylim'][0])
				self.grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)

				rospy.loginfo('UAS {} will start in grid position: {}.'.format(self.uid, self.grid_pos))
				rospy.loginfo('UAS {} has loaded boundary bisectors.'.format(self.uid))

			obstacles = rospy.get_param('/obstacles', dict())
			if len(obstacles):
				for hn, hv in obstacles.items():
					segments = []
					self.holes[hn] = np.array(hv, dtype=float)

					hv = np.array(hv, dtype=float)

					for i in range(len(hv)):
						j = (i + 1) % len(hv)

						middle = (hv[j] + hv[i]) * 0.5
						direction = hv[j] - hv[i]

						normal = np.array([-direction[1], direction[0]], dtype=float)
						normal /= np.linalg.norm(normal)
						segments.append((normal, middle, "H_{}_{}_{}".format(hn, i, j)))
						# self.holes[hn].append(hv[i])

					self.hole_segments.extend(segments)

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

			self.footprint_size = rospy.get_param('/footprint_size', 1.)
			rospy.loginfo('UAS {0} will have coverage footprint size {1} cells in grid.'.format(self.uid, self.footprint_size))

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
		self.flight_state_pub.publish(FlightStateCompressed(self.uid, 
															self.position[0], 
															self.position[1], 
															self.velocity[0], 
															self.velocity[1], 
															self.converged))

		# self.sense_info_pub.publish(SensingInfo(self.position[0], self.position[1], self.grid_pos[0], self.grid_pos[1], self.uid))

	def broadcast_cell(self):
		vcell_data = []
		for v in self.voronoi_cell:
			vcell_data.extend(list(v))

		vcell_data.extend(list(self.voronoi_cell[0]))

		self.voronoi_cell_pub.publish(VCell(self.uid, 2, vcell_data))

	def compute_peer_repulsion(self):
		force = np.zeros(2)

		alpha = 3.0
		eps = 8.0
		sigma = 12.0

		for _, fs in self.nb_states.items():
			repulsion = fs['pos'] - self.position
			magnitude = np.linalg.norm(repulsion)

			# if magnitude >= 15.:
			# 	continue

			repulsion /= magnitude
			multiplier = - eps * (2 * (sigma ** 4) / (magnitude ** 5) - (sigma ** 2) / (magnitude ** 3))
			force += alpha * multiplier * repulsion

		return force

	def compute_goal(self):
		if len(self.voronoi_cell) >= 3:
			self.voronoi_cell.append(self.voronoi_cell[0])
			centroid = get_centroid(self.voronoi_cell, self.uid)
			self.set_goal(centroid)

	def compute_goal_attraction(self):
		gamma = 8.0

		force = self.goal - self.position
		magnitude = np.linalg.norm(force)
		force /= magnitude

		return gamma * force

	def compute_boundary_respulsion(self):
		force = np.zeros(2)

		for bseg in self.boundary:
			seg_n, seg_p, _ = bseg

			if np.all(seg_n.dot(self.position) <= seg_n.dot(seg_p) + 0.1):
				force += np.dot(seg_p - self.position, seg_n) * 1.5

		return force

	def compute_obstacle_repulsion(self):
		force = np.zeros(2)

		for hseg in self.hole_segments:
			seg_n, seg_p, _ = hseg

			if np.all(seg_n.dot(self.position) <= seg_n.dot(seg_p) + 0.1):
				force += np.dot(self.position - seg_p, seg_n)

		return force

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

	def compute_voronoi_constraints(self):
		try:
			constraints, values = [], []

			for nbid, fs in self.nb_states.items():
				normal = (fs['pos'] - self.position).round(2)
				middle = ((fs['pos'] + self.position) * 0.5).round(2)
				constraints.append(normal)
				values.append((middle.dot(normal)).round(2))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)

			return A_cell, b_cell

		except Exception as e:
			print(traceback.format_exc())
			return None, None

	def compute_bisectors(self):
		try:
			vbisectors = []
			constraints, values = [], []
			tol = 0.1

			for nbid, fs in self.nb_states.items():
				normal = (fs['pos'] - self.position).round(2)
				middle = ((fs['pos'] + self.position) * 0.5).round(2)
				constraints.append(normal)
				values.append((middle.dot(normal)).round(2))
				vbisectors.append((normal, middle, "D_{}{}".format(self.uid, nbid)))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)
			self.voronoi_cell = []
			cell_graph = VGraph()


			####################################################################################
			# Voronoi Bisectors vs. Voronoi Bisectors ##########################################
			self.logger.write('UAS {} - Voronoi Construction:\n'.format(self.uid))
			for i in range(len(vbisectors) - 1):
				n_i, m_i, vb_name = vbisectors[i]
				d_i = m_i.dot(n_i)

				for j in range(i + 1, len(vbisectors)):
					n_j, m_j, b_name = vbisectors[j]
					d_j = m_j.dot(n_j)

					try:
						A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
						b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
						p = np.linalg.solve(A_, b_).round(2)

					except np.linalg.LinAlgError:
						continue

					except:
						print(traceback.format_exc())
						continue

					inside_check = is_point_valid(self.boundary_vertices, p, self.holes)
					feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)

					if inside_check and feasibility_check:
						cell_graph.add_edge(vb_name, b_name, VEdge("P_{}_{}".format(vb_name, b_name), p))
						self.logger.write('\t- {} X {} = {} - Success!\n'.format(vb_name, b_name, p))

					else:
						self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(vb_name, b_name, p, inside_check, feasibility_check))
			####################################################################################


			####################################################################################
			# Voronoi Bisectors vs. Boundary Segments ##########################################
			for i in range(len(vbisectors)):
				n_i, m_i, vb_name = vbisectors[i]
				d_i = m_i.dot(n_i)

				for j in range(len(self.boundary)):
					n_j, m_j, b_name = self.boundary[j]
					d_j = m_j.dot(n_j)

					try:
						A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
						b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
						p = np.linalg.solve(A_, b_).round(2)

					except np.linalg.LinAlgError:
						continue

					except:
						print(traceback.format_exc())
						continue

					k = (j + 1) % len(self.boundary_vertices)
					inside_check = onSegment(self.boundary_vertices[j], p, self.boundary_vertices[k])
					feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)

					if inside_check and feasibility_check:
						cell_graph.add_edge(vb_name, b_name, VEdge("P_{}_{}".format(vb_name, b_name), p))
						self.logger.write('\t- {} X {} = {} - Success!\n'.format(vb_name, b_name, p))

					else:
						self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(vb_name, b_name, p, inside_check, feasibility_check))
			####################################################################################


			####################################################################################
			# Voronoi Bisectors vs. Hole Segments ##############################################
			for i in range(len(vbisectors)):
				n_i, m_i, vb_name = vbisectors[i]
				d_i = m_i.dot(n_i)

				for j in range(len(self.hole_segments)):
					n_j, m_j, h_name = self.hole_segments[j]
					d_j = m_j.dot(n_j)

					try:
						A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
						b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
						p = np.linalg.solve(A_, b_).round(2)

					except np.linalg.LinAlgError:
						continue

					except:
						print(traceback.format_exc())
						continue


					hole_name_parsed = h_name.split('_')
					hole_seg_start, hole_seg_end = (self.holes[hole_name_parsed[1]][int(hole_name_parsed[2])], 
													self.holes[hole_name_parsed[1]][int(hole_name_parsed[3])])

					inside_check = onSegment(hole_seg_start, p, hole_seg_end)
					feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)

					if inside_check and feasibility_check:
						cell_graph.add_edge(vb_name, h_name, VEdge("P_{}_{}".format(vb_name, h_name), p))
						self.logger.write('\t- {} X {} = {} - Success!\n'.format(vb_name, h_name, p))

					else:
						self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(vb_name, h_name, p, inside_check, feasibility_check))
			####################################################################################


			####################################################################################
			# Voronoi Bisectors vs. Hole Vertices ##############################################
			for hn, hv in self.holes.items():
				for i in range(len(hv)):
					j = (i + 1) % len(hv)
					k = (i - 1 + len(hv)) % len(hv)

					v = hv[i]
					feasibility, diff = self.check_feasibility(A_cell, b_cell, v)

					if feasibility:
						first, second = "H_{}_{}_{}".format(hn, k, i), "H_{}_{}_{}".format(hn, i, j)
						cell_graph.add_edge(first, second, VEdge("HBND_{}".format(i), v))
						self.logger.write('\t- {} X {} = {} - Success!\n'.format(first, second, v))

					else:
						self.logger.write('\t- Hole {} vertex {} - Failure! ||| INFEASIBLE, diff: {}\n'.format(hn, v, diff))
			####################################################################################


			####################################################################################
			# Adding Boundary Vertices to Cell Graph, If Necessary #############################
			for i in range(len(self.boundary_vertices)):
				j = (i + 1) % len(self.boundary_vertices)
				k = (i - 1 + len(self.boundary_vertices)) % len(self.boundary_vertices)

				v = self.boundary_vertices[i]
				feasibility, diff = self.check_feasibility(A_cell, b_cell, v)

				if feasibility:
					first, second = "B_{}{}".format(k, i), "B_{}{}".format(i, j)
					cell_graph.add_edge(first, second, VEdge("BND_{}".format(i), v))
					self.logger.write('\t- {} X {} = {} - Success!\n'.format(first, second, v))

				else:
					self.logger.write('\t- Boundary vertex {} - Failure! ||| INFEASIBLE, diff: {}\n'.format(v, diff))
			####################################################################################


			self.voronoi_cell = cell_graph.traverse()

			self.logger.write("\n***********\n************\n")
			self.logger.write("Voronoi cell of UAS {}:\n".format(self.uid))
			for v in self.voronoi_cell:
				self.logger.write("\t{}\n".format(v))
			self.logger.write("\n***********\n************\n")

			A_iq = matrix(A_cell, tc='d')
			b_iq = matrix(b_cell, tc='d')
			self.dump_voronoi_cell()
			self.broadcast_cell()

			return A_iq, b_iq
		except Exception as e:
			print(traceback.format_exc())

	def compute_bisectors2(self):
		try:
			vbisectors = []
			constraints, values = [], []
			tol = 0.1

			for nbid, fs in self.nb_states.items():
				normal = (fs['pos'] - self.position).round(2)
				middle = ((fs['pos'] + self.position) * 0.5).round(2)
				constraints.append(normal)
				values.append((middle.dot(normal)).round(2))
				vbisectors.append((normal, middle, "D_{}{}".format(self.uid, nbid)))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)
			self.voronoi_cell = []
			cell_graph = VGraph()


			####################################################################################
			# Voronoi Bisectors vs. Voronoi Bisectors ##########################################
			self.logger.write('UAS {} - Voronoi Construction:\n'.format(self.uid))
			for i in range(len(vbisectors) - 1):
				n_i, m_i, vb_name = vbisectors[i]
				d_i = m_i.dot(n_i)

				vector_i = self.position - m_i
				angle_i = np.arctan2(vector_i[1], vector_i[0])
				descending_order = (angle_i >= np.pi * 0.5 or angle_i < - np.pi * 0.5)
				v_node_i = VNode(vb_name, m_i, sorting=True, descending=descending_order)

				for j in range(i + 1, len(vbisectors)):
					n_j, m_j, b_name = vbisectors[j]
					d_j = m_j.dot(n_j)

					vector_j = self.position - m_j
					angle_j = np.arctan2(vector_j[1], vector_j[0])
					descending_order = (angle_j >= np.pi * 0.5 or angle_j < - np.pi * 0.5)
					v_node_i = VNode(vb_name, m_i, sorting=True, descending=descending_order)

					try:
						A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
						b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
						p = np.linalg.solve(A_, b_).round(2)

					except np.linalg.LinAlgError:
						continue

					except:
						print(traceback.format_exc())
						continue

					inside_check = is_point_valid(self.boundary_vertices, p, self.holes)
					feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)

					if inside_check and feasibility_check:
						# cell_graph.add_edge(vb_name, b_name, VEdge("P_{}_{}".format(vb_name, b_name), p))
						# cell_graph.add_edge(b_name, vb_name, VEdge("P_{}_{}".format(vb_name, b_name), p))
						cell_graph.add_edge(v_node_i, v_node_j, VEdge("P_{}_{}".format(vb_name, b_name), p))
						cell_graph.add_edge(v_node_j, v_node_i, VEdge("P_{}_{}".format(vb_name, b_name), p))
						self.logger.write('\t- {} <--X--> {} = {} - Success!\n'.format(vb_name, b_name, p))

					else:
						self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(vb_name, b_name, p, inside_check, feasibility_check))
			####################################################################################


			####################################################################################
			# Voronoi Bisectors vs. Boundary Segments ##########################################
			for i in range(len(vbisectors)):
				n_i, m_i, vb_name = vbisectors[i]
				d_i = m_i.dot(n_i)

				for j in range(len(self.boundary)):
					n_j, m_j, b_name = self.boundary[j]
					d_j = m_j.dot(n_j)

					try:
						A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
						b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
						p = np.linalg.solve(A_, b_).round(2)

					except np.linalg.LinAlgError:
						continue

					except:
						print(traceback.format_exc())
						continue

					k = (j + 1) % len(self.boundary_vertices)
					inside_check = onSegment(self.boundary_vertices[j], p, self.boundary_vertices[k])
					feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)

					if inside_check and feasibility_check:
						start_feasible = np.dot(n_i, self.boundary_vertices[j]) <= d_i + 0.5
						end_feasible = np.dot(n_i, self.boundary_vertices[k]) <= d_i + 0.5

						if start_feasible:
							cell_graph.add_edge(b_name, vb_name, VEdge("P_{}_{}".format(vb_name, b_name), p))
							self.logger.write('\t- {} --X--> {} = {} - Success!\n'.format(b_name, vb_name, p))

						elif end_feasible:
							cell_graph.add_edge(vb_name, b_name, VEdge("P_{}_{}".format(vb_name, b_name), p))
							self.logger.write('\t- {} --X--> {} = {} - Success!\n'.format(vb_name, b_name, p))

						else:
							rospy.logwarn("UAS {} - Should this be executed at all ??".format(self.uid))

					else:
						self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(vb_name, b_name, p, inside_check, feasibility_check))
			####################################################################################


			####################################################################################
			# Voronoi Bisectors vs. Hole Segments ##############################################
			# for i in range(len(vbisectors)):
			# 	n_i, m_i, vb_name = vbisectors[i]
			# 	d_i = m_i.dot(n_i)

			# 	for j in range(len(self.hole_segments)):
			# 		n_j, m_j, h_name = self.hole_segments[j]
			# 		d_j = m_j.dot(n_j)

			# 		try:
			# 			A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
			# 			b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
			# 			p = np.linalg.solve(A_, b_).round(2)

			# 		except np.linalg.LinAlgError:
			# 			continue

			# 		except:
			# 			print(traceback.format_exc())
			# 			continue


			# 		hole_name_parsed = h_name.split('_')
			# 		hole_seg_start, hole_seg_end = (self.holes[hole_name_parsed[1]][int(hole_name_parsed[2])], 
			# 										self.holes[hole_name_parsed[1]][int(hole_name_parsed[3])])

			# 		inside_check = onSegment(hole_seg_start, p, hole_seg_end)
			# 		feasibility_check, diff = self.check_feasibility(A_cell, b_cell, p)

			# 		if inside_check and feasibility_check:
			# 			cell_graph.add_edge(vb_name, h_name, VEdge("P_{}_{}".format(vb_name, h_name), p))
			# 			self.logger.write('\t- {} X {} = {} - Success!\n'.format(vb_name, h_name, p))

			# 		else:
			# 			self.logger.write('\t- {} X {} = {} - Failure! ||| INSIDE: {} ||| FEASIBLE: {}\n'.format(vb_name, h_name, p, inside_check, feasibility_check))
			####################################################################################


			####################################################################################
			# Voronoi Bisectors vs. Hole Vertices ##############################################
			# for hn, hv in self.holes.items():
			# 	for i in range(len(hv)):
			# 		j = (i + 1) % len(hv)
			# 		k = (i - 1 + len(hv)) % len(hv)

			# 		v = hv[i]
			# 		feasibility, diff = self.check_feasibility(A_cell, b_cell, v)

			# 		if feasibility:
			# 			first, second = "H_{}_{}_{}".format(hn, k, i), "H_{}_{}_{}".format(hn, i, j)
			# 			cell_graph.add_edge(first, second, VEdge("HBND_{}".format(i), v))
			# 			self.logger.write('\t- {} X {} = {} - Success!\n'.format(first, second, v))

			# 		else:
			# 			self.logger.write('\t- Hole {} vertex {} - Failure! ||| INFEASIBLE, diff: {}\n'.format(hn, v, diff))
			####################################################################################


			####################################################################################
			# Adding Boundary Vertices to Cell Graph, If Necessary #############################
			for i in range(len(self.boundary_vertices)):
				j = (i + 1) % len(self.boundary_vertices)
				k = (i - 1 + len(self.boundary_vertices)) % len(self.boundary_vertices)

				v = self.boundary_vertices[i]
				feasibility, diff = self.check_feasibility(A_cell, b_cell, v)

				if feasibility:
					first, second = "B_{}{}".format(k, i), "B_{}{}".format(i, j)
					cell_graph.add_edge(first, second, VEdge("BND_{}".format(i), v))
					self.logger.write('\t- {} X {} = {} - Success!\n'.format(first, second, v))

				else:
					self.logger.write('\t- Boundary vertex {} - Failure! ||| INFEASIBLE, diff: {}\n'.format(v, diff))
			####################################################################################


			# self.voronoi_cell = cell_graph.traverse()
			self.voronoi_cell = cell_graph.traverse(self.logger)

			self.logger.write("\n***********\n************\n")
			self.logger.write("Voronoi cell of UAS {}:\n".format(self.uid))
			for v in self.voronoi_cell:
				self.logger.write("\t{}\n".format(v))
			self.logger.write("\n***********\n************\n")

			A_iq = matrix(A_cell, tc='d')
			b_iq = matrix(b_cell, tc='d')
			self.dump_voronoi_cell()
			self.broadcast_cell()

			return A_iq, b_iq
		except Exception as e:
			print(traceback.format_exc())

	def set_goal(self, g):
		change = np.linalg.norm(g - self.goal)

		if change < 0.01:
			self.converged_count += 1

		if self.converged_count >= 25:
			self.converged = True

		else:
			self.converged = False

		self.prev_goal = self.goal
		self.goal = np.array(g, dtype=float)

		if self.coverage_state == 0:
			rospy.loginfo('UAS {} goal set to {} (change: {}) - (count: {}).'.format(self.uid, 
																					 self.goal, 
																					 change, 
																					 self.converged_count))

	def set_coverage_start_goal(self):
		# guidance_vector = np.array([np.cos(self.heading + np.pi / 2.), np.sin(self.heading + np.pi / 2.)], dtype=float)
		guidance_vector = np.array([np.cos(np.pi / 2.), np.sin(np.pi / 2.)], dtype=float)
		cell_vertices_as_grid = map(lambda v: real_to_grid(v, self.grid_offset, self.grid_resolution), self.voronoi_cell)
		cell_vertices_offsetted = map(lambda v: grid_to_real(v, self.grid_offset, self.grid_resolution), cell_vertices_as_grid)
		utility_values = map(lambda v: np.dot(np.array(v, dtype=float), guidance_vector), cell_vertices_offsetted)
		# coverage_start_as_grid = real_to_grid(self.voronoi_cell[0], self.grid_offset, self.grid_resolution)
		# coverage_start_offsetted = grid_to_real(coverage_start_as_grid, self.grid_offset, self.grid_resolution)
		# self.set_goal(coverage_start_offsetted)
		# self.set_goal(cell_vertices_offsetted[np.argmax(utility_values)])
		return cell_vertices_offsetted[np.argmax(utility_values)]

	def get_converged_mass(self):
		cell_copy = list(self.voronoi_cell)
		cell_copy.append(cell_copy[0])
		return np.abs(get_area(cell_copy))

	def compute_goal_from_immediate_frontier(self, eligibles):
		maxIdx, maxVal = 0, eligibles[0][0]

		for i in range(1, len(eligibles)):
			if eligibles[i][0] > maxVal:
				# self.frontier[eligibles[maxIdx][1]] = maxVal

				maxVal = eligibles[i][0]
				maxIdx = i

			# else:
			# 	self.frontier[eligibles[i][1]] = eligibles[i][0]

		return eligibles[maxIdx]

	def solve_step(self, direct=False):
		try:
			v_next = self.velocity

			if not direct:
				if len(self.voronoi_cell) >= 3:
					self.voronoi_cell.append(self.voronoi_cell[0])
					centroid = get_centroid(self.voronoi_cell, self.uid)
					self.set_goal(centroid)

			v_next = self.goal - self.position
			_norm = np.linalg.norm(v_next)

			if _norm > self.args['vmax']:
				v_next *= self.args['vmax'] / _norm

			return v_next

			# return np.zeros(2)
		except Exception as e:
			print(traceback.format_exc())
			return np.zeros(2)

	def solve_step2(self):
		try:
			self.grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)
			actions = valid_actions2(self.grid_dims, self.grid_pos, footprint_size=self.footprint_size)

			constraints, values = [], []
			tol = 0.1

			for _, fs in self.nb_states.items():
				normal = (fs['pos'] - self.position).round(2)
				middle = ((fs['pos'] + self.position) * 0.5).round(2)
				constraints.append(normal)
				values.append((middle.dot(normal)).round(2))

			A_cell = np.array(constraints, dtype=float)
			b_cell = np.array(values, dtype=float)

			eligible_goals = []
			goal_values = []
			self.logger.write("***\nUAS {} - Eligible goals from position {} / {}:\n".format(self.uid, self.grid_pos, self.position))
			for a in actions:
				cand_grid_pos = (self.grid_pos[0] + a[0], self.grid_pos[1] + a[1])
				cand_real_pos = grid_to_real(cand_grid_pos, self.grid_offset, self.grid_resolution)

				feasibility, diff = self.check_feasibility(A_cell, b_cell, cand_real_pos)
				is_unexplored = (self.local_grid_map[cand_grid_pos] == -1)
				# is_inside = is_inside_polygon(self.boundary_vertices, cand_real_pos)
				is_inside = is_point_valid(self.boundary_vertices, cand_real_pos, self.holes)

				self.logger.write("\t Candidate: ({:.3f}, {:.3f}) - IN: {}, FSB: {}\n".format(cand_real_pos[0], cand_real_pos[1], is_inside, feasibility))

				if is_inside and feasibility:
				# if is_inside and feasibility and is_unexplored:
					goal_vector = cand_real_pos - self.position
					heading_vector = np.array([np.cos(self.heading), np.sin(self.heading)], dtype=float)
					gain = info_gain(self.local_grid_map, cand_grid_pos)
					# value = goal_vector.dot(heading_vector) - self.local_grid_map[cand_grid_pos] + a[2]
					value = goal_vector.dot(heading_vector) + gain + a[2] + int(is_unexplored) * 1000
					# value = goal_vector.dot(heading_vector) + a[2]
					# value = goal_vector[0] * np.cos(self.heading) + goal_vector[1] * np.sin(self.heading) + a[2]
					self.logger.write("\t ({:.3f}, {:.3f}) with utility value: {:.3f}\n\n".format(cand_real_pos[0], cand_real_pos[1], value))

					eligible_goals.append((value, cand_real_pos))

			if len(eligible_goals) > 0:
				next_goal = self.compute_goal_from_immediate_frontier(eligible_goals)
				self.set_goal(next_goal[1])

				self.logger.write("\t\t Chosen: ({:.3f}, {:.3f})\n".format(next_goal[1][0], next_goal[1][1]))

				v_next = self.goal - self.position
				_norm = np.linalg.norm(v_next)

				if _norm > self.args['vmax']:
					v_next *= self.args['vmax'] / _norm

				return v_next

			elif len(self.frontier) > 0:
				# There were no eligible goals left to be explored in the immediate frontier.
				# Look for the skipped locations in the past.
				rospy.logwarn('UAS {} has resorted to old frontier.'.format(self.uid))
				pass

			rospy.logwarn('UAS {} has no eligible goals!'.format(self.uid))
			return np.zeros(2)

		except Exception as e:
			print(traceback.format_exc())
			return np.zeros(2)

	def solve_step3(self, A_cell, b_cell, first=False):
		try:
			if A_cell is None or b_cell is None:
				print("NONE type cell constraint matrix and value vector is given!")
				return np.zeros(2)

			self.grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)
			actions = valid_actions2(self.grid_dims, self.grid_pos, footprint_size=self.footprint_size)
			# actions = valid_actions2(self.grid_dims, self.grid_pos, footprint_size=self.footprint_size, log=(self.uid == 0))

			eligible_goals = []
			goal_values = []
			self.logger.write("***\nUAS {} - Eligible goals from position {} / {}:\n".format(self.uid, self.grid_pos, self.position))
			for a in actions:
				cand_grid_pos = (self.grid_pos[0] + a[0], self.grid_pos[1] + a[1])
				cand_real_pos = grid_to_real(cand_grid_pos, self.grid_offset, self.grid_resolution)

				feasibility, diff = self.check_feasibility(A_cell, b_cell, cand_real_pos)
				is_unexplored = (self.local_grid_map[cand_grid_pos] == -1)
				is_inside = is_point_valid(self.boundary_vertices, cand_real_pos, self.holes)

				self.logger.write("\t Candidate: ({:.3f}, {:.3f}) - IN: {}, FSB: {}\n".format(cand_real_pos[0], cand_real_pos[1], is_inside, feasibility))

				if is_inside and feasibility:
				# if is_inside and feasibility and is_unexplored:
					goal_vector = cand_real_pos - self.position
					heading_vector = np.array([np.cos(self.heading), np.sin(self.heading)], dtype=float)

					# Left biased
					guidance_vector = np.array([np.cos(np.pi / 2.), np.sin(np.pi / 2.)], dtype=float)
					# guidance_vector = np.array([np.cos(self.heading + np.pi / 2.), np.sin(self.heading + np.pi / 2.)], dtype=float)

					# Right biased
					# guidance_vector = np.array([np.cos(self.heading - np.pi / 2.), np.sin(self.heading - np.pi / 2.)], dtype=float)

					# value = 2 * goal_vector.dot(heading_vector) * int(is_unexplored) + goal_vector.dot(guidance_vector) + a[2]
					if first:
						heading_vector = guidance_vector

					value = goal_vector.dot(heading_vector) + int(is_unexplored) * 1000

					self.logger.write("\t ({:.3f}, {:.3f}) with utility value: {:.3f}\n\n".format(cand_real_pos[0], cand_real_pos[1], value))
					eligible_goals.append((value, cand_real_pos))

			if len(eligible_goals) > 0:
				next_goal = self.compute_goal_from_immediate_frontier(eligible_goals)
				self.set_goal(next_goal[1])

				self.logger.write("\t\t Chosen: ({:.3f}, {:.3f})\n".format(next_goal[1][0], next_goal[1][1]))

				v_next = self.goal - self.position
				_norm = np.linalg.norm(v_next)

				if _norm > self.args['vmax']:
					v_next *= self.args['vmax'] / _norm

				return v_next

			elif len(self.frontier) > 0:
				# There were no eligible goals left to be explored in the immediate frontier.
				# Look for the skipped locations in the past.
				rospy.logwarn('UAS {} has resorted to old frontier.'.format(self.uid))
				pass

			rospy.logwarn('UAS {} has no eligible goals!'.format(self.uid))
			return np.zeros(2)

		except Exception as e:
			print(traceback.format_exc())
			return np.zeros(2)

	def solve_step_by_force(self):
		exerted_force = np.zeros(2)
		peer_rep = np.zeros(2)
		goal_attr = np.zeros(2)
		bnd_rep = np.zeros(2)
		obs_rep = np.zeros(2)

		self.compute_goal()
		peer_rep = self.compute_peer_repulsion()
		# goal_attr = self.compute_goal_attraction()
		bnd_rep = self.compute_boundary_respulsion()
		# obs_rep = self.compute_obstacle_repulsion()

		self.logger.write("\n*****\nForces on UAS {}:\n".format(self.uid))
		self.logger.write("\tPeer repulsion    : {}\n".format(peer_rep))
		self.logger.write("\tGoal attraction   : {}\n".format(goal_attr))
		self.logger.write("\tBoundary repulsion: {}\n".format(bnd_rep))
		self.logger.write("\tObstacle repulsion: {}\n".format(obs_rep))
		self.logger.write("*****\n")

		exerted_force = peer_rep + goal_attr + bnd_rep + obs_rep

		magnitude = np.linalg.norm(exerted_force)
		if magnitude > self.args['vmax']:
			exerted_force *= self.args['vmax'] / magnitude

		return exerted_force

	def vel_pub(self):
		vel_cmd = Twist()
		vel_cmd.linear.x = self.velocity[0]
		vel_cmd.linear.y = self.velocity[1]
		self.velocity_pub(vel_cmd)

	def execute_step(self, v_next):
		prev_grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)

		next_position = self.position + v_next / self.args['rate']
		if is_point_valid(self.boundary_vertices, next_position, self.holes):
			self.position = next_position

		else:
			pass
			# Do nothing for now

		# self.position = self.position + v_next / self.args['rate']
		self.grid_pos = real_to_grid(self.position, self.grid_offset, self.grid_resolution)
		self.velocity = v_next
		self.heading = np.arctan2(v_next[1], v_next[0])

		# if np.linalg.norm(self.prev_goal - self.goal) >= 1.:
			# prev_grid_pos = real_to_grid(self.prev_goal, self.grid_offset, self.grid_resolution)

		if self.coverage_state == 2:
			if self.local_grid_map[prev_grid_pos] == -1:
				self.local_grid_map[prev_grid_pos] = self.uid

		if self.grid_pos != prev_grid_pos:
			self.sense_info_pub.publish(SensingInfo(self.prev_goal[0], self.prev_goal[1], prev_grid_pos[0], prev_grid_pos[1], self.uid, 1))

			# pgx, pgy = prev_grid_pos
			# fs = int(self.footprint_size)
			# self.local_grid_map[max(0, pgx - fs):min(self.grid_dims[0], pgx + fs + 1), max(0, pgy - fs):min(self.grid_dims[1], pgy + fs + 1)] = self.uid

			# for i in range(max(0, pgx - fs), min(self.grid_dims[0], pgx + fs + 1)):
			# 	for j in range(max(0, pgy - fs), min(self.grid_dims[1], pgy + fs + 1)):
			# 		if self.local_grid_map[i, j] == -1:
			# 			self.local_grid_map[i, j] = self.uid

			# self.sense_info_pub.publish(SensingInfo(self.prev_goal[0], self.prev_goal[1], pgx, pgy, self.uid, self.footprint_size))