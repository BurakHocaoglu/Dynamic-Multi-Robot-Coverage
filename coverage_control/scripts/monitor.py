#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

import os
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from coverage_control.msg import FlightState, FlightStateCompressed, VCell

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5)]

class Monitor:

	def __init__(self, paramFile):
		self.valid = False
		self.flight_states = dict()
		self.voronoi_cells = dict()
		self.past_motions = dict()
		self.uas_x_coords = None
		self.uas_y_coords = None
		self.arena = None
		self.args = dict()
		self.setup(paramFile)

		self.flight_state_sub = rospy.Subscriber('/flight_states', FlightStateCompressed, self.flight_state_cb, queue_size=80)
		self.voronoi_cell_sub = rospy.Subscriber('/voronoi_cells', VCell, self.voronoi_cell_cb, queue_size=80)
		# self.collision_checker = rospy.Timer(rospy.Duration(1.0), self.check_collision)

		self.figure = plt.figure()
		self.axes = self.figure.add_subplot(1, 1, 1)
		self.animation_func = animation.FuncAnimation(self.figure, self.animate_movement, interval=100)

	def flight_state_cb(self, msg):
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

		# rospy.loginfo('Monitor detected UAS {0} at ({1}, {2}) with velocity ({3}, {4})!'.format(msg.id, msg.x, msg.y, msg.vx, msg.vy))

	def voronoi_cell_cb(self, msg):
		idx, cell = 0, []
		while idx < len(msg.data) - msg.dim:
			cell.append(np.array(msg.data[idx:idx + msg.dim], dtype=float))
			idx += msg.dim

		self.voronoi_cells[msg.id] = cell

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

		if self.arena is not None:
			self.axes.add_patch(self.arena)

		if not self.valid:
			return

		for uid, ufs in self.flight_states.items():
			pos, vel = ufs['pos'], ufs['vel']
			heading = np.arctan2(vel[1], vel[0])
			robot_color = globals()['__COLORS'][uid + 1]
			self.axes.quiver(pos[0], pos[1], np.cos(heading), np.sin(heading), color=robot_color)
			self.axes.add_artist(plt.Circle(tuple(pos), 2., color=robot_color))
			self.axes.plot(self.past_motions[uid], color=robot_color)

			robot_cell = self.voronoi_cells.get(uid)
			if robot_cell is not None and len(robot_cell) > 2:
				self.axes.add_patch(plt.Polygon(robot_cell, alpha=0.4, color=robot_color))

	def setup(self, paramFile):
		try:
			if paramFile is None:
				rospy.logerr('NoneType parameter file name!')
				sys.exit(-1)

			params = None
			with open(paramFile, 'r') as PF:
				params = yaml.load(PF)

			if not params:
				rospy.logerr('Empty parameter structure!')
				sys.exit(-1)

			self.args['uas_count'] = params.get('uas_count')
			if self.args['uas_count'] is None:
				rospy.logwarn('Monitor could not find: uas_count!')

			else:
				self.uas_x_coords = np.zeros(self.args['uas_count'])
				self.uas_y_coords = np.zeros(self.args['uas_count'])
				rospy.loginfo('Monitor will supervise {} agents.'.format(self.args['uas_count']))

			_boundary = params.get('boundary')
			if _boundary is None:
				rospy.logwarn('Monitor could not find: boundary!')

			else:
				x_comps = map(lambda a: a[0], _boundary)
				y_comps = map(lambda a: a[1], _boundary)
				self.args['xlim'] = [min(x_comps), max(x_comps)]
				self.args['ylim'] = [min(y_comps), max(y_comps)]
				rospy.loginfo('Monitor has set space limits from X({0}) to Y({1}).'.format(self.args['xlim'], self.args['ylim']))

				self.args['boundary'] = _boundary
				self.arena = plt.Polygon(angular_sort(np.mean(self.args['boundary'], axis=0), self.args['boundary']), color=(0, 0, 0), fill=False)
				rospy.loginfo('Monitor will observe the arena of:')
				for v in self.args['boundary']:
					print('\t{}'.format(v))

		except Exception as e:
			print(traceback.format_exc())

		finally:
			rospy.loginfo('Monitor has been setup.')

	def is_ready(self):
		count = self.args.get('uas_count')
		if count is None:
			return False

		return len(self.flight_states) == count