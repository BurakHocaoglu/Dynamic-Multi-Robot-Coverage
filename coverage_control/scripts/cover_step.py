#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import matplotlib.pyplot as plt

#####################################################
### Helper data structures ##########################

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
#####################################################

#####################################################
### Boundary vertices ###############################

# boundary = [np.array((0., 40.), dtype=float), 
# 			np.array((-40., 0.), dtype=float),
# 			np.array((0., -40.), dtype=float),
# 			np.array((40., 0.), dtype=float)]

boundary = [np.array((0., 40.), dtype=float), 
			np.array((-20., 10.), dtype=float),
			np.array((-50., 0.), dtype=float),
			np.array((0., -20.), dtype=float), 
			np.array((30., -10.), dtype=float),
			np.array((20., 0.), dtype=float),
			np.array((30., 30.), dtype=float)]

#####################################################

#####################################################
### Hole vertices ###################################

# hole = [np.array([10., 0.], dtype=float), 
# 		np.array([0., -10.], dtype=float), 
# 		np.array([10., -20.], dtype=float)]

hole = []

#####################################################

#####################################################
### Member locations ################################

# neighbours = [np.array([-10., 10.], dtype=float), 
# 			  np.array([10., 10.], dtype=float)]

neighbours = [np.array([-10., 10.], dtype=float), 
			  np.array([10., 20.], dtype=float)]

# position = np.array([-10., -10.], dtype=float)

position = np.array([10., -10.], dtype=float)

#####################################################

#####################################################
### Candidate locations #############################

# candidates = [np.array([-20., -10.], dtype=float), 
# 			  np.array([-20., 10.], dtype=float), 
# 			  np.array([-10., -20.], dtype=float), 
# 			  np.array([20., -10.], dtype=float)]

candidates = [np.array([-20., -10.], dtype=float), 
			  np.array([0., -10.], dtype=float), 
			  np.array([20., -10.], dtype=float), 
			  np.array([14., 0.], dtype=float)]

#####################################################

boundary_constraints = []
constraints = []
values = []

bounding_polygon = Polygon([Point(v[0], v[1]) for v in boundary])

# Compute boundary constraints
boundary.append(boundary[0])
for i in range(len(boundary) - 1):
	middle = (boundary[i] + boundary[i + 1]) * 0.5
	d = boundary[i + 1] - boundary[i]
	normal = np.array([d[1], -d[0]])
	boundary_constraints.append((normal, middle))
	# constraints.append(normal)
	# values.append(middle.dot(normal))

# Compute voronoi constraints
voronoi_dividers = []
for n in neighbours:
	middle = (position + n) * 0.5
	normal = n - position
	voronoi_dividers.append((normal, middle))
	constraints.append(normal)
	values.append(middle.dot(normal))

# Costruct the figure
figure = plt.figure()
ax = figure.add_subplot(1, 1, 1)

# Plot the task space
vertices = boundary + hole
xs, ys = zip(*vertices)
ax.set_xlim(min(xs) - 10., max(xs) + 10.)
ax.set_ylim(min(ys) - 10., max(ys) + 10.)
ax.set_aspect('equal')

# Plot boundary polygon
boundary_polygon = plt.Polygon(boundary, alpha=0.9, color=(0., 0., 0.99), fill=False)
ax.add_patch(boundary_polygon)

# Plot agent location
agent = plt.Circle(tuple(position), 1., color=(0.99, 0.45, 0.))
ax.add_patch(agent)

# Plot boundary normals
for i in range(len(boundary_constraints)):
	normal, mid = boundary_constraints[i]
	heading = np.arctan2(normal[1], normal[0])
	ax.quiver(mid[0], mid[1], np.cos(heading), np.sin(heading), color=(0.99, 0., 0.99))

# Plot voronoi normals
for i in range(len(voronoi_dividers)):
	normal, mid = voronoi_dividers[i]
	heading = np.arctan2(normal[1], normal[0])
	ax.quiver(mid[0], mid[1], np.cos(heading), np.sin(heading), color=(0.99, 0., 0.99))

# Compute Ax <= b format
A = np.array(constraints, dtype=float)
b = np.array(values, dtype=float)

def test(p, A, b):
	product = A.dot(p)
	print("-------- Candidate: ({}, {}):".format(p[0], p[1]))
	for i in range(len(b)):
		print("\t Computed: {} - Target: {} - Diff: {} - Result: {}".format(product[i], b[i], product[i] - b[i], product[i] <= b[i] + 0.1))
	print("--------\n")
	return np.all(product <= b + 0.1)

# eval_start = time.time()
# Plot candidate locations
for i in range(len(candidates)):
	cand_color = (0., 0., 0.)

	if (bounding_polygon.contains(Point(candidates[i][0], candidates[i][1])) and 
		np.all(A.dot(candidates[i]) <= b + 0.1)):
		# The candidate is valid
		cand_color = (0., 0.99, 0.)
		print("candidate {} is valid".format(candidates[i]))

	else:
		pass

	cand = plt.Circle(tuple(candidates[i]), 1., color=cand_color)
	ax.add_patch(cand)

# print("Took {} seconds to compute all candidates.".format(time.time() - eval_start))

plt.show()