#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *
# from map_manager import *

import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from coverage_control.msg import FlightState, FlightStateCompressed, VCell, SensingInfo
# from coverage_control.msg import FlightState, VCell

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5)]

class Monitor:

	def __init__(self):
		self.valid = False
		self.flight_states = dict()
		self.voronoi_cells = dict()
		self.past_motions = dict()
		self.uas_x_coords = None
		self.uas_y_coords = None
		self.arena = None
		self.args = dict()
		# self.map_manager = None
		self.grid_map = None
		self.grid_dims = None
		self.grid_image = None
		self.grid_image_file = None
		self.grid_drawer = None
		self.grid_resolution = 1.
		self.grid_offset = None
		self.setup()

		self.flight_state_sub = rospy.Subscriber('/flight_states', FlightStateCompressed, self.flight_state_cb, queue_size=80)
		self.voronoi_cell_sub = rospy.Subscriber('/voronoi_cells', VCell, self.voronoi_cell_cb, queue_size=80)
		self.sensing_info_sub = rospy.Subscriber('/sensing_info', SensingInfo, self.sensing_info_cb, queue_size=80)
		# self.collision_checker = rospy.Timer(rospy.Duration(1.0), self.check_collision)

		self.figure = plt.figure()
		self.axes = self.figure.add_subplot(1, 1, 1)
		self.animation_func = animation.FuncAnimation(self.figure, self.animate_movement, interval=100)

	def flight_state_cb(self, msg):
		try:
			self.uas_x_coords[msg.id] = msg.x
			self.uas_y_coords[msg.id] = msg.y

			if self.flight_states.get(msg.id) is None:
				self.flight_states[msg.id] = {'pos': np.array([msg.x, msg.y], dtype=float), 'vel': np.array([msg.vx, msg.vy], dtype=float)}
				self.past_motions[msg.id] = [np.array([msg.x, msg.y], dtype=float)]

			else:
				self.flight_states[msg.id]['pos'] = np.array([msg.x, msg.y], dtype=float)
				self.flight_states[msg.id]['vel'] = np.array([msg.vx, msg.vy], dtype=float)

				if len(self.past_motions[msg.id]) > 1:
					curr_pos = np.array([msg.x, msg.y], dtype=float)
					prev_pos = self.past_motions[msg.id][-1]
					if np.linalg.norm(curr_pos - prev_pos) < 0.5:
						self.past_motions[msg.id].append(curr_pos)

			# if self.grid_offset is not None:
			# 	im_color = color_real_to_int(globals()['__COLORS'][msg.id + 1])
			# 	print('im_color: {}'.format(im_color))
			# 	grid_pos = real_to_grid((msg.x, msg.y), self.grid_offset, self.grid_resolution)
			# 	print('elliptic: {}'.format((grid_pos[0], grid_pos[1], grid_pos[0] - 10, grid_pos[1] - 10)))
			# 	self.grid_drawer.ellipse((grid_pos[0], grid_pos[1], grid_pos[0] - 10, grid_pos[1] - 10), fill=im_color, outline=im_color)

		except Exception as e:
			print(traceback.format_exc())

	def sensing_info_cb(self, msg):
		if self.grid_offset is not None:
			im_color = color_real_to_int(globals()['__COLORS'][int(msg.data) + 1])
			# grid_pos = real_to_grid((msg.x, msg.y), self.grid_offset, self.grid_resolution)
			self.grid_drawer.ellipse((msg.grid_x - 5, msg.grid_y - 5, msg.grid_x + 5, msg.grid_y + 5), fill=im_color, outline=im_color)
			# self.grid_drawer.point((msg.grid_x, msg.grid_y), fill=im_color)
			# self.grid_drawer.point((msg.grid_y, msg.grid_x), fill=im_color)

	def voronoi_cell_cb(self, msg):
		try:
			idx, cell = 0, []
			while idx < len(msg.data) - msg.dim:
				cell.append(np.array(msg.data[idx:idx + msg.dim], dtype=float))
				idx += msg.dim

			self.voronoi_cells[msg.id] = cell
		except Exception as e:
			print(traceback.format_exc())

		# rospy.loginfo('Monitor detected UAS {0} at cell:'.format(msg.id))
		# for vertex in cell:
		# 	print('\t{}'.format(vertex))

	def check_collision(self, event):
		if not self.valid:
			return

		if self.uas_x_coords is None or self.uas_y_coords is None:
			rospy.logerr('Monitor has coordinate matrices as NoneType!')
			return

		XX1, XX2 = np.meshgrid(self.uas_x_coords, self.uas_x_coords)
		YY1, YY2 = np.meshgrid(self.uas_y_coords, self.uas_y_coords)
		pairwise_dists = np.sqrt((XX2 - XX1) ** 2 + (YY2 - YY1) ** 2)
		R, C = pairwise_dists.shape

		for i in range(R):
			for j in range(C):
				if j < i and pairwise_dists[i, j] <= 2.0:
					rospy.logerr("Collision between (agent %d) and (agent %d)" % (i + 1, j + 1))

	def animate_movement(self, i):
		self.axes.clear()
		self.axes.set_xlim(self.args['xlim'][0] - 5, self.args['xlim'][1] + 5)
		self.axes.set_ylim(self.args['ylim'][0] - 5, self.args['ylim'][1] + 5)
		self.axes.set_aspect('equal')

		if self.arena is not None:
			self.axes.add_patch(self.arena)

		if not self.valid:
			return

		for uid, ufs in self.flight_states.items():
			pos, vel = ufs['pos'], ufs['vel']
			heading = np.arctan2(vel[1], vel[0])
			robot_color = globals()['__COLORS'][uid + 1]
			self.axes.quiver(pos[0], pos[1], np.cos(heading), np.sin(heading), color=robot_color)
			self.axes.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
			self.axes.plot(self.past_motions[uid], color=robot_color)

			robot_cell = self.voronoi_cells.get(uid)
			if robot_cell is not None and len(robot_cell) > 2:
				# robot_cell.append(robot_cell[0])
				# cell_xcoords, cell_ycoords = zip(*robot_cell)
				# self.axes.plot(cell_xcoords, cell_ycoords, color=robot_color)
				self.axes.add_patch(plt.Polygon(robot_cell, alpha=0.4, color=robot_color))

	def setup(self):
		try:
			self.args['uas_count'] = rospy.get_param('/uas_count', None)
			if self.args['uas_count'] is None:
				rospy.logwarn('Monitor could not find: uas_count!')

			else:
				self.uas_x_coords = np.zeros(self.args['uas_count'])
				self.uas_y_coords = np.zeros(self.args['uas_count'])
				rospy.loginfo('Monitor will supervise {} agents.'.format(self.args['uas_count']))

			_boundary = rospy.get_param('/boundary', None)
			if _boundary is None:
				rospy.logwarn('Monitor could not find: boundary!')

			else:
				x_comps, y_comps = zip(*_boundary)
				self.args['xlim'] = [min(x_comps), max(x_comps)]
				self.args['ylim'] = [min(y_comps), max(y_comps)]
				rospy.loginfo('Monitor has set space limits from X({0}) to Y({1}).'.format(self.args['xlim'], self.args['ylim']))

				self.args['boundary'] = _boundary
				sorted_boundary = angular_sort(np.mean(self.args['boundary'], axis=0), self.args['boundary'])
				self.arena = plt.Polygon(sorted_boundary, color=(0, 0, 0), fill=False)
				rospy.loginfo('Monitor will observe the arena of:')
				for v in self.args['boundary']:
					print('\t{}'.format(v))

				# retrieve obstacles here
				# self.map_manager = MapManager("coverage_map", _boundary)
				self.grid_resolution = rospy.get_param('/grid_res', 1.)
				self.grid_map = create_map(sorted_boundary, self.grid_resolution, dict())
				self.grid_dims = self.grid_map.shape
				self.grid_image = Image.new('RGB', (self.grid_dims[0] + 5, self.grid_dims[0] + 5), (255, 255, 255))
				self.grid_drawer = ImageDraw.Draw(self.grid_image)
				self.grid_offset = (self.args['xlim'][0], self.args['ylim'][0])

				sorted_grid_coords = []
				print('Image pixel coordinates:')
				for v in sorted_boundary:
					grid_coord = tuple(real_to_grid(v, self.grid_offset, self.grid_resolution))
					print('\t{} -> {}'.format(v, grid_coord))
					sorted_grid_coords.append(grid_coord)

				self.grid_drawer.polygon(sorted_grid_coords, outline=(0, 0, 0))

			logdir_prefix = rospy.get_param('/logdir_prefix', None)
			if logdir_prefix is None:
				cwd = os.getcwd()
				rospy.logwarn('UAS {0} could not find: logdir_prefix! Setting to default {1}.'.format(self.uid, cwd))
				logdir_prefix = cwd

			else:
				self.args['logdir_prefix'] = logdir_prefix
				rospy.loginfo('Monitor will log to the path {}.'.format(self.args['logdir_prefix']))

			stamp = datetime.datetime.now()
			logdir = '{0}{1}'.format(self.args['logdir_prefix'], stamp.strftime('%b_%d_%Y_%H_%M'))

			if not os.path.exists(logdir):
				rospy.logwarn("Path {} does not exist. Creating...".format(logdir))
				os.makedirs(logdir)

			self.grid_image_file = logdir + '/coverage_map.png'

		except Exception as e:
			print(traceback.format_exc())

		finally:
			rospy.loginfo('Monitor has been setup.')

	def save_grid_image(self):
		self.grid_image.save(self.grid_image_file)

	def is_ready(self):
		count = self.args.get('uas_count')
		if count is None:
			return False

		return len(self.flight_states) == count