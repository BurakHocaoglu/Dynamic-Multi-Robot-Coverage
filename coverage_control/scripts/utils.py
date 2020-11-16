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

	rospy.loginfo('UAS {0} has voronoi mass {1}.'.format(uid, area))

	# Calculate centroid of voronoi cell
	Cx, Cy = 0, 0
	for i in range(len(polygon) - 1):
		x_i, y_i = polygon[i]
		x_j, y_j = polygon[i + 1]
		product = (x_i * y_j - x_j * y_i)
		Cx += (x_i + x_j) * product
		Cy += (y_i + y_j) * product

	return np.array([Cx, Cy], dtype=float) / (6. * area)