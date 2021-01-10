#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import yaml
import time
import rospy
import datetime
import traceback
import numpy as np

from enum import Enum
from PIL import Image, ImageDraw

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import StreamRate, SetMode, CommandBool, CommandTOL

# class ItemNotImplementedError(Exception):

# 	def __init__(self, value=""):
# 		self.value = "{} not implemented!".format(value)

# 	def __str__(self):
# 		return repr(self.value)

# class ConstraintType(Enum):
# 	"""
# 	LINE constraint is composed of:
# 	* Normal vector
# 	* A point that the line passes through
# 	"""
# 	LINE=1

# 	"""
# 	LINE_SEGMENT constraint is composed of:
# 	"""
# 	LINE_SEGMENT=2

# 	"""
# 	"""
# 	CIRCLE=3

# 	"""
# 	"""
# 	FUNCTION=4

def is_in_space(xlims, ylims, p, tol):
	return (xlims[0] - tol <= p[0] <= xlims[1] + tol and
			ylims[0] - tol <= p[1] <= ylims[1] + tol)

def point_in_polygon(point, polygon):
	oddNodes = False
	current = None
	pass

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

def get_centroid(polygon, uid):
	### SOURCE: https://en.wikipedia.org/wiki/Centroid

	# rospy.loginfo('UAS {} called get_centroid with:'.format(uid))
	# for v in polygon:
	# 	print('\t{}'.format(v))

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

def prune_voronoi_cell(cell):
	pass

def line_and_segment_intersection(line, segment):
	pass

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

	for obs in obstacles:
		pass

	return grid

def print_grid_to_png(grid, pngfile):
	try:
		png = Image.fromarray(grid)
		png.save(pngfile)
		print('Grid is printed.')
	except Exception as e:
		print(traceback.format_exc())

class Point:

	def __init__(self, x, y):
		self.x = x
		self.y = y

class Polygon:

	def __init__(self, points):
		self.points = points

	@property
	def edges(self):
		edge_list = []
		for i, p in enumerate(self.points):
			p1 = p
			p2 = self.points[(i + 1) % len(self.points)]
			edge_list.append((p1, p2))

		return edge_list

	def contains(self, point):
		import sys

		_huge = sys.float_info.max
		_eps = 0.00001

		inside = False
		for edge in self.edges:
			A, B = edge[0], edge[1]

			if A.y > B.y:
				A, B = B, A

			if point.y == A.y or point.y == B.y:
				point.y += _eps

			if (point.y > B.y or point.y < A.y or point.x > max(A.x, B.x)):
				continue

			if point.x < min(A.x, B.x):
				inside = not inside
				continue

			try:
				m_edge = (B.y - A.y) / (B.x - A.x)
			except ZeroDivisionError:
				m_edge = _huge

			try:
				m_point = (point.y - A.y) / (point.x - A.x)
			except ZeroDivisionError:
				m_point = _huge

			if m_point >= m_edge:
				inside = not inside
				continue

		return inside