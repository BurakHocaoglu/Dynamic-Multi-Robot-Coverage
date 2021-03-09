#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import yaml
import time
import rospy
import datetime
import traceback
import threading
import numpy as np

from enum import Enum
from collections import deque
from PIL import Image, ImageDraw
from shapely.geometry import Point, Polygon

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, SetMode, CommandBool, CommandTOL

def is_in_space(xlims, ylims, p, tol):
	return (xlims[0] - tol <= p[0] <= xlims[1] + tol and
			ylims[0] - tol <= p[1] <= ylims[1] + tol)

def angle_in_2pi(v):
	return np.arctan2(v[1], v[0])

def angular_sort(reference, vertices):
	vectors = [p - reference for p in vertices]
	indexed_angles = [(angle_in_2pi(vectors[i]), i) for i in range(len(vectors))]
	indexed_angles.sort()
	return [vertices[i] for _, i in indexed_angles]

def get_index_of_angular_insert(vertices, vertex, reference):
	i = 0

	angle = angle_in_2pi(vertex - reference)
	while i < len(vertices):
		if angle < angle_in_2pi(vertices[i] - reference):
			break

		i += 1

	return i

def check_angle_integrity(vertices, reference):
	for i in range(len(vertices) - 1):
		if angle_in_2pi(vertices[i] - reference) > angle_in_2pi(vertices[i + 1] - reference):
			return False

	return True

def get_area(polygon):
	area = 0
	for i in range(len(polygon) - 1):
		x_i, y_i = polygon[i]
		x_j, y_j = polygon[i + 1]
		area += x_i * y_j - x_j * y_i

	return area * 0.5

def get_centroid(polygon, uid):
	### SOURCE: https://en.wikipedia.org/wiki/Centroid

	# Calculate area with Shoelace Formula
	area = 0
	for i in range(len(polygon) - 1):
		x_i, y_i = polygon[i]
		x_j, y_j = polygon[i + 1]
		area += x_i * y_j - x_j * y_i

	area *= 0.5

	# rospy.loginfo('UAS {0} has voronoi mass {1}.'.format(uid, area))

	# Calculate centroid of voronoi cell
	Cx, Cy = 0, 0
	for i in range(len(polygon) - 1):
		x_i, y_i = polygon[i]
		x_j, y_j = polygon[i + 1]
		product = (x_i * y_j - x_j * y_i)
		Cx += (x_i + x_j) * product
		Cy += (y_i + y_j) * product

	return np.array([Cx, Cy], dtype=float) / (6. * area)

def real_to_grid(p, offset, resolution):
	return (int(round((p[0] - offset[0]) * resolution)), 
			int(round((p[1] - offset[1]) * resolution)))

def grid_to_real(p, offset, resolution):
	return np.array([float(p[0]) / resolution + offset[0], 
					 float(p[1]) / resolution + offset[1]])

def color_real_to_int(color):
	return (int(color[0] * 255), 
			int(color[1] * 255), 
			int(color[2] * 255))

def create_map(vertices, resolution, obstacles):
	if vertices is None or len(vertices) < 3:
		rospy.logwarn('Not enough vertices were provided for map creation!')
		sys.exit(-1)

	xcoords, ycoords = zip(*(vertices))
	xmin, xmax = min(xcoords), max(xcoords)
	ymin, ymax = min(ycoords), max(ycoords)
	width = int((xmax - xmin) / resolution)
	height = int((ymax - ymin) / resolution)
	grid = - np.ones((height, width))

	# for obs in obstacles:
	# 	pass

	return grid

def print_grid_to_png(grid, pngfile):
	try:
		png = Image.fromarray(grid)
		png.save(pngfile)
		print('Grid is printed.')
	except Exception as e:
		print(traceback.format_exc())

class Action(Enum):
	WEST = (0, -1, 1)
	EAST = (0, 1, 1)
	NORTH = (-1, 0, 1)
	SOUTH = (1, 0, 1)
	NORTH_WEST = (-1, -1, np.sqrt(2))
	NORTH_EAST = (-1, 1, np.sqrt(2))
	SOUTH_WEST = (1, -1, np.sqrt(2))
	SOUTH_EAST = (1, 1, np.sqrt(2))

	@property
	def cost(self):
		return self.value[2]

	@property
	def delta(self):
		return (self.value[0], self.value[1])

def valid_actions(grid_size, current_node):
	_actions = list(Action)
	n, m = grid_size[0] - 1, grid_size[1] - 1
	r, c = current_node

	if r - 1 < 0:
		_actions.remove(Action.NORTH)

	if r + 1 > n:
		_actions.remove(Action.SOUTH)

	if c - 1 < 0:
		_actions.remove(Action.WEST)

	if c + 1 > m:
		_actions.remove(Action.EAST)

	if (r - 1 < 0 or c - 1 < 0):
		_actions.remove(Action.NORTH_WEST)

	if (r - 1 < 0 or c + 1 > m):
		_actions.remove(Action.NORTH_EAST)

	if (r + 1 > n or c - 1 < 0):
		_actions.remove(Action.SOUTH_WEST)

	if (r + 1 > n or c + 1 > m):
		_actions.remove(Action.SOUTH_EAST)

	return _actions

def valid_actions2(grid_size, current_node, footprint_size=1, log=False):
	cr, cc = current_node
	gr, gc = grid_size

	if log:
		print("This agent is at {}".format(current_node))

	actions = []
	for i in range(cr - footprint_size, cr + footprint_size + 1):
		for j in range(cc - footprint_size, cc + footprint_size + 1):
			# if log:
			# 	print("Checking ({}, {})...".format(i, j))
			if i < 0 or i >= gr or j < 0 or j >= gc:
				continue

			if i == cr and j == cc:
				continue

			actions.append((i - cr, j - cc, np.hypot(cr - i, cc - j)))

	if log:
		print("Actions for this agent:")
		for a in actions:
			print(a)
		print("---------------")

	return actions

def info_gain(grid, query_cell):
	gain = 0
	qr, qc = query_cell

	for i in range(qr - 1, qr + 2):
		for j in range(qc - 1, qc + 2):
			if grid[i, j] == -1:
				gain += 1

	return gain

# class Region:

# 	def __init__(self, bVertices=[], hVertices=dict()):
# 		self.boundaryVertices = bVertices
# 		self.holeVertices = hVertices

# def onSegment(p, q, r):
# 	if ((q[0] <= max(p[0], r[0])) &
# 		(q[0] >= min(p[0], r[0])) &
# 		(q[1] <= max(p[1], r[1])) &
# 		(q[1] >= min(p[1], r[1]))):
# 		return True

# 	return False

# def orientation(p, q, r):
# 	val = (((q[1] - p[1]) * (r[0] - q[0])) -
# 		   ((q[0] - p[0]) * (r[1] - q[1])))

# 	if val == 0:
# 		return 0

# 	if val > 0:
# 		return 1

# 	else:
# 		return 2

# def doIntersect(p1, q1, p2, q2):
# 	o1 = orientation(p1, q1, p2)
# 	o2 = orientation(p1, q1, q2)
# 	o3 = orientation(p2, q2, p1)
# 	o4 = orientation(p2, q2, q1)

# 	if (o1 != o2) and (o3 != o4):
# 		return True

# 	if (o1 == 0) and (onSegment(p1, p2, q1)):
# 		return True

# 	if (o2 == 0) and (onSegment(p1, q2, q1)):
# 		return True

# 	if (o3 == 0) and (onSegment(p2, p1, q2)):
# 		return True

# 	if (o4 == 0) and (onSegment(p2, q1, q2)):
# 		return True

# 	return False

# def is_inside_polygon(points, p):
# 	n = len(points)

# 	if n < 3:
# 		return False

# 	xs, ys = zip(*points)
# 	xmax, xmin = max(xs), min(xs)
# 	ymax, ymin = max(ys), min(ys)

# 	# if p[0] == 0:
# 	# 	return ymin <= p[1] <= ymax

# 	# if p[1] == 0:
# 	# 	return xmin <= p[0] <= xmax

# 	if not ((xmin <= p[0] <= xmax) and (ymin <= p[1] <= ymax)):
# 		return False

# 	extreme = (1e4, p[1])
# 	count = i = 0

# 	while True:
# 		next = (i + 1) % n

# 		if (doIntersect(points[i], points[next], p, extreme)):
# 			if orientation(points[i], p, points[next]) == 0:
# 				return onSegment(points[i], p, points[next])

# 			count += 1

# 		i = next

# 		if (i == 0):
# 			break

# 	return (count % 2 == 1)

# def is_point_valid(bVertices, p, hVertices=None):
# 	if hVertices is not None:
# 		for _, hole in hVertices.items():
# 			if is_inside_polygon(hole, p):
# 				return False

# 	return is_inside_polygon(bVertices, p)

def onSegment(p, q, r):
	if ((q[0] > max(p[0], r[0])) or
		(q[0] < min(p[0], r[0])) or
		(q[1] > max(p[1], r[1])) or
		(q[1] < min(p[1], r[1]))):
		return False

	point = np.array(q, dtype=float)
	normal = np.array([p[1] - r[1], r[0] - p[0]], dtype=float)
	middle = np.array([p[0] + r[0], p[1] + r[1]], dtype=float) * 0.5
	return np.dot(point - middle, normal) / np.linalg.norm(normal) < 0.1

def isPointContained(poly, q):
	# print(type(poly))
	for i in xrange(len(poly.exterior.coords) - 1):
		p, r = poly.exterior.coords[i], poly.exterior.coords[i + 1]

		# if onSegment(p, q.coords[0], r):
		if onSegment(p, q, r):
			return True

	return poly.contains(Point(q[0], q[1]))

def is_point_valid(poly, q, obs=dict()):
	for _, o in obs.items():
		if isPointContained(o, q):
			return False

	return isPointContained(poly, q)

class VEdge:

	def __init__(self, name, point):
		self.name = name
		self.point = point

class VNode:

	def __init__(self, name, point, sorting=False, descending=True):
		self.name = name
		self.point = point
		self.neighbours = dict()
		self.neighbour_order = dict()
		self.is_descending = descending
		self.to_be_sorted = sorting
		self.direction = descending

	def add_neighbour(self, edge, vnode):
		connection_pair = (edge, vnode)

		if self.neighbours.get(edge.name) is None:
			self.neighbours[edge.name] = connection_pair

	def sort_neighbours(self, logger=None):
		if self.to_be_sorted:
			order = [(np.sum(conn[0].point - self.point), conn) for _, conn in self.neighbours.items()]
			order.sort(reverse=self.is_descending)

			self.neighbour_order = [c[1] for c in order]

			if logger is not None:
				logger.write("\n\tTraversal order over {}:\n".format(self.name))
				for c in self.neighbour_order:
					logger.write("\tEdge: {}, Node: {}\n".format(c[0].name, c[1].name))
				logger.write("\n")

		else:
			self.neighbour_order = [c for _, c in self.neighbours.items()]

	def get_next(self, last_visited):
		pass

class VGraph:

	def __init__(self, ofs, res):
		self.nodes = dict()
		self.offset = ofs
		self.resolution = res
		# self.intersection_table = dict()

	def add_node(self, v):
		if self.nodes.get(v.name) is None:
			self.nodes[v.name] = v

	def add_edge(self, v1, v2, edge, force=False):
		# grid_point = real_to_grid(edge.point, self.offset, self.resolution)
		# if not force and self.intersection_table.get(grid_point) is not None:
		# 	return

		if self.nodes.get(v1.name) is None:
			self.nodes[v1.name] = v1

		if self.nodes.get(v2.name) is None:
			self.nodes[v2.name] = v2

		self.nodes[v1.name].add_neighbour(edge, v2)
		# self.intersection_table[grid_point] = True

	def preprocessing(self, logger):
		for _, node in self.nodes.items():
			node.sort_neighbours(logger)

	def traverse(self, logger=None):
		if len(self.nodes) < 3:
			return []

		traversal = []
		stack = deque()
		start = self.nodes.keys()[0]
		stack.append(self.nodes[start])
		visited = dict()
		last_visited_edge = None

		while len(stack) > 0:
			vnode = stack.pop()

			if logger is not None:
				logger.write("\t *** Checking {}\n".format(vnode.name))

			for (edge, nb_node) in self.nodes[vnode.name].neighbour_order:
				if logger is not None:
					logger.write("\t\t **** Checking neighbour of {}: {} - {}\n".format(vnode.name, edge.name, nb_node.name))

				if visited.get(edge.name) is None:
					stack.append(nb_node)
					visited[edge.name] = True

					if logger is not None:
						logger.write("\t\t\t ***** Traversed node {}, {}\n".format(edge.name, edge.point))

					traversal.append(edge)
					break

		geometric_visited = dict()
		geo_traversal = []

		for e in traversal:
			e_visited = False

			for e_name, edge in geometric_visited.items():
				if np.linalg.norm(np.array(e.point) - np.array(edge.point)) <= 0.5:
					e_visited = True
					break

			if not e_visited:
				geometric_visited[e.name] = e
				geo_traversal.append(np.array(e.point))

		return geo_traversal