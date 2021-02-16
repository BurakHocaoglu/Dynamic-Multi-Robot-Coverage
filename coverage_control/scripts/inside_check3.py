#!/usr/bin/env python
# -*- coding: utf-8 -*-

def is_convex(polygon):
	flag = 0
	z = 0

	if len(polygon) < 3:
		return None

	for i in range(len(polygon)):
		j = (i + 1) % len(polygon)
		k = (i + 2) % len(polygon)
		z = (polygon[j][0] - polygon[i][0]) * (polygon[k][1] - polygon[j][1])
		z -= (polygon[j][1] - polygon[i][1]) * (polygon[k][0] - polygon[j][0])

		if z < 0:
			flag |= 1

		elif z > 0:
			flag |= 2

		else:
			return False

	if flag:
		return True

	return None

def encloses_point(polygon, p):
	lit = []
	for v in polygon:
		lit.append((v[0] - p[0], v[1] - p[1]))

	indices = range(- len(lit), 1)

	# if is_convex(lit):
	# 	orientation = None

	# 	for i in indices:
	# 		a = polygon[i]
	# 		b = polygon[i + 1]
	# 		test = a[0] * (b[1] - a[1]) - a[1] * (b[0] - a[0]) < 0

	# 		if orientation is None:
	# 			orientation = test

	# 		elif test is not orientation:
	# 			return False

	# 	return True

	hit_odd = False
	p1x, p1y = lit[0]

	for i in indices[1:]:
		p2x, p2y = lit[i]

		if 0 > min(p1y, p2y):

			if 0 <= max(p1y, p2y):

				if 0 <= max(p1x, p2x):

					if p1y != p2y:
						xinters = - p1y * (p2x - p1x) / (p2y - p1y) + p1x

						if p1x == p2x or 0 <= xinters:
							hit_odd = not hit_odd

		p1x, p1y = p2x, p2y

	return hit_odd

from shapely.geometry import Polygon, Point

if __name__ == "__main__":
	poly = [(0., 30.), (-20., 10.), (-10., -10.), (0., 10.), (20., 0.), (20., 20.)]
	# poly = Polygon([(0., 30.), (-20., 10.), (-10., 10.), (0., 10.), (20., 0.), (20., 20.)])

	p1 = (4., 2.)
	p2 = (-10., 10.)
	p3 = (-20., 0.)
	p4 = (-10., 0.)
	p5 = (0., 10.)
	p6 = (10., 15.)

	# p1 = Point(4., 2.)
	# p2 = Point(-10., 10.)
	# p3 = Point(-20., 0.)
	# p4 = Point(-10., 0.)
	# p5 = Point(0., 10.)
	# p6 = Point(10., 15.)

	print "Convexity:", is_convex(poly)

	print str(encloses_point(poly, p1)) # Should be False
	print str(encloses_point(poly, p2)) # Should be True
	print str(encloses_point(poly, p3)) # Should be False
	print str(encloses_point(poly, p4)) # Should be True
	print str(encloses_point(poly, p5)) # Examine further, vertex case; for now False
	print str(encloses_point(poly, p6)) # Should be True

	# print str(poly.contains(p1)) # Should be False
	# print str(poly.contains(p2)) # Should be True
	# print str(poly.contains(p3)) # Should be False
	# print str(poly.contains(p4)) # Should be True
	# print str(poly.contains(p5)) # Examine further, vertex case ???
	# print str(poly.contains(p6)) # Should be True

################################################################################
################################################################################
################################################################################
################################################################################

# from shapely.geometry import Point
# from shapely.geometry.polygon import Polygon

# point = Point(-10., -9.)
# polygon = Polygon([(0., 30.), (-20., 10.), (-10., -10.), (0., 10.), (20., 0.), (20., 20.)])
# print(polygon.contains(point))

################################################################################
################################################################################
################################################################################
################################################################################

# import matplotlib.path as mplPath
# import numpy as np

# poly = [(0., 30.), (-20., 10.), (-10., -10.), (0., 10.), (20., 0.), (20., 20.)]
# # poly = [(0, 0), (0, 1), (1, 1)]
# bbPath = mplPath.Path(np.array(poly, dtype=float))

# p1 = (4., 2.)
# p2 = (-10., 10.)
# p3 = (-20., 0.)
# p4 = (-10., 0.)
# p5 = (0., 10.)
# p6 = (10., 15.)

# print str(bbPath.contains_point(p1)) # Should be False
# print str(bbPath.contains_point(p2)) # Should be True
# print str(bbPath.contains_point(p3)) # Should be False
# print str(bbPath.contains_point(p4)) # Should be True
# print str(bbPath.contains_point(p5)) # Examine further, vertex case ???
# print str(bbPath.contains_point(p6)) # Should be True

# print str(bbPath.contains_point((0.5, 0.5)))

################################################################################
################################################################################
################################################################################
################################################################################

# from PyQt5.QtCore import QPoint, QPointF, Qt
# from PyQt5.QtGui import QPolygon, QPolygonF

# poly = [(0., 30.), (-20., 10.), (-10., -10.), (0., 10.), (20., 0.), (20., 20.)]
# qpnts = [QPointF(*v) for v in poly]
# qpolyf = QPolygonF()

# for qpnt in qpnts:
# 	qpolyf.append(qpnt)

# # p1 = QPointF(4., 2.)
# # p2 = QPointF(-10., 10.)
# # p3 = QPointF(-20., 0.)
# # p4 = QPointF(-9., 0.)
# # p5 = QPointF(0., 10.)
# # p6 = QPointF(10., 15.)
# p7 = QPointF(-10., 1.)

# print "Odd Even Fill:"
# # print str(qpolyf.containsPoint(p1, Qt.OddEvenFill)) # Should be False
# # print str(qpolyf.containsPoint(p2, Qt.OddEvenFill)) # Should be True
# # print str(qpolyf.containsPoint(p3, Qt.OddEvenFill)) # Should be False
# # print str(qpolyf.containsPoint(p4, Qt.OddEvenFill)) # Should be True
# # print str(qpolyf.containsPoint(p5, Qt.OddEvenFill)) # Examine further, vertex case ???
# # print str(qpolyf.containsPoint(p6, Qt.OddEvenFill)) # Should be True
# print str(qpolyf.containsPoint(p7, Qt.OddEvenFill)) # Should be True
# print "---"

# print "Winding Fill:"
# # print str(qpolyf.containsPoint(p1, Qt.WindingFill)) # Should be False
# # print str(qpolyf.containsPoint(p2, Qt.WindingFill)) # Should be True
# # print str(qpolyf.containsPoint(p3, Qt.WindingFill)) # Should be False
# # print str(qpolyf.containsPoint(p4, Qt.WindingFill)) # Should be True
# # print str(qpolyf.containsPoint(p5, Qt.WindingFill)) # Examine further, vertex case ???
# # print str(qpolyf.containsPoint(p6, Qt.WindingFill)) # Should be True
# print str(qpolyf.containsPoint(p7, Qt.WindingFill)) # Should be True