#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

if __name__ == '__main__':
	boundary = [[-60., 0.], [0., 43.6], [60., 0.], [37.1, 70.5], [97.1, 114.1], [22.9, 114.1], 
				[0., 184.7], [-24.6, 114.1], [-97.1, 114.1], [-37.6, 69.]]

	# concave = Polygon([Point(-20., 100.),
	# 				   Point(-80., 40.),
	# 				   Point(-60., -40.),
	# 				   Point(-20., -20.),
	# 				   Point(40., -60.),
	# 				   Point(60., 60.)])

	concave = Polygon([Point(v[0], v[1]) for v in boundary])

	tp1 = Point(-73., 83)
	tp2 = Point(-42., 24.)
	tp3 = Point(14., -35.)
	tp4 = Point(76., -43.)

	print('Test Point 1 inside polygon? {}'.format(concave.contains(tp1)))
	print('Test Point 2 inside polygon? {}'.format(concave.contains(tp2)))
	print('Test Point 3 inside polygon? {}'.format(concave.contains(tp3)))
	print('Test Point 4 inside polygon? {}'.format(concave.contains(tp4)))