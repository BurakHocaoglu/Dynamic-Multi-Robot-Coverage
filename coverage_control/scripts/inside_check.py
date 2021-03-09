#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from shapely.geometry import Polygon, Point

# class Point:

# 	def __init__(self, x, y):
# 		self.x = x
# 		self.y = y

# class Polygon:

# 	def __init__(self, points):
# 		self.points = points

# 	@property
# 	def edges(self):
# 		edge_list = []
# 		for i, p in enumerate(self.points):
# 			p1 = p
# 			p2 = self.points[(i + 1) % len(self.points)]
# 			edge_list.append((p1, p2))

# 		return edge_list

# 	def contains(self, point):
# 		import sys

# 		_huge = sys.float_info.max
# 		_eps = 0.00001

# 		inside = False
# 		for edge in self.edges:
# 			A, B = edge[0], edge[1]

# 			if A.y > B.y:
# 				A, B = B, A

# 			if point.y == A.y or point.y == B.y:
# 				point.y += _eps

# 			if (point.y > B.y or point.y < A.y or point.x > max(A.x, B.x)):
# 				print("[continued] P: ({}, {}) - E: ({}, {}) - ({}, {})".format(point.x, point.y, A.x, A.y, B.x, B.y))
# 				continue

# 			if point.x < min(A.x, B.x):
# 				inside = not inside
# 				print("[continued] P: ({}, {}) - E: ({}, {}) - ({}, {}) - IN: {}".format(point.x, point.y, A.x, A.y, B.x, B.y, inside))
# 				continue

# 			try:
# 				m_edge = (B.y - A.y) / (B.x - A.x)
# 			except ZeroDivisionError:
# 				m_edge = _huge

# 			try:
# 				m_point = (point.y - A.y) / (point.x - A.x)
# 			except ZeroDivisionError:
# 				m_point = _huge

# 			if m_point >= m_edge:
# 				inside = not inside
# 				continue

# 		return inside

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
	for i in xrange(len(poly.exterior.coords) - 1):
		p, r = poly.exterior.coords[i], poly.exterior.coords[i + 1]

		if onSegment(p, q.coords[0], r):
			return True

	return poly.contains(q)

def isPointValid(poly, q, obs=dict()):
	for _, o in obs.items():
		if isPointContained(o, q):
			return False

	return isPointContained(poly, q)

if __name__ == '__main__':
	# boundary = [[-60., 0.], [0., 43.6], [60., 0.], [37.1, 70.5], [97.1, 114.1], [22.9, 114.1], 
	# 			[0., 184.7], [-24.6, 114.1], [-97.1, 114.1], [-37.6, 69.]]

	# boundary = [[0., 40.], [-40., 0.], [0., -40.], [40., 0.]]
	boundary = [[8., 8.], [8., -6.], [-10., 0.]]

	test_points = [Point(-6., 0.), 
				   Point(0., 0.), 
				   Point(0., 4.),
				   Point(2., -2.),
				   Point(8.44, -2.05),
				   Point(2., 8.),
				   Point(-6., 4.),
				   Point(8., 8.), 
				   Point(4.85, 0.7),
				   Point(0.76, 0.72),
				   Point(2., 2.),
				   Point(3.57, 2.),
				   Point(-0.98, -1.01),
				   Point(4., 0.), 
				   Point(1.9, 0.8),
				   Point(1.14, 0.23)]

	test_obstacles = {
		'obs1': [(2., 0.), (0., 0.), (0., 2.), (2., 2.), (0.76, 0.72)],
		'obs2': [(4., 2.), (4., 0.), (6., 0.), (6., 2.)]
	}

	poly = Polygon(boundary)
	obs_poly = dict([(name, Polygon(hole)) for name, hole in test_obstacles.items()])

	poly_x_coords, poly_y_coords = zip(*boundary)
	p_x_min, p_x_max = min(poly_x_coords), max(poly_x_coords)
	p_y_min, p_y_max = min(poly_y_coords), max(poly_y_coords)

	test_x_coords = map(lambda p_: p_.coords[0][0], test_points)
	test_y_coords = map(lambda p_: p_.coords[0][1], test_points)
	t_x_min, t_x_max = min(test_x_coords), max(test_x_coords)
	t_y_min, t_y_max = min(test_y_coords), max(test_y_coords)

	x_min, x_max = min(t_x_min, p_x_min), max(t_x_max, p_x_max)
	y_min, y_max = min(t_y_min, p_y_min), max(t_y_max, p_y_max)

	figure = plt.figure()
	ax = figure.add_subplot(1, 1, 1)
	ax.set_xlim(x_min - 5., x_max + 5.)
	ax.set_ylim(y_min - 5., y_max + 5.)
	ax.set_aspect('equal')

	ax.add_patch(plt.Polygon(boundary, color=(0, 0, 0), fill=False))
	for _, hole in test_obstacles.items():
		ax.add_patch(plt.Polygon(hole, color=(0, 0, 0), fill=False))

	for p in test_points:
		# test = poly.contains(p)
		test = isPointValid(poly, p, obs_poly)
		c = (0., 0., 0.)

		if test:
			c = (0., 0.99, 0.)

		else:
			c = (0.99, 0., 0.)

		pos = p.coords[0]
		ax.add_artist(plt.Circle(pos, (x_max - x_min) * 0.02, color=c))

	plt.show()