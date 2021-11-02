#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import traceback
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

import shapely.geometry as sg

from geometry_msgs.msg import Point

from coverage_control2.srv import GetMetricPartition

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

if __name__ == "__main__":
	rospy.init_node("metric_partition_test", anonymous=False)

	limits = None
	region_patch = None
	obstacle_patches = dict()

	exp_region = rospy.get_param("/coverage_boundary", [])
	if len(exp_region) < 3:
		rospy.logerr("Region of interest has less than 3 vertices!")
		sys.exit(1)

	else:
		xcoords, ycoords = zip(*exp_region)
		xmin, xmax = min(xcoords), max(xcoords)
		ymin, ymax = min(ycoords), max(ycoords)
		limits = (xmin, ymin, xmax, ymax)
		region_patch = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))
		exp_region = sg.Polygon(exp_region).buffer(-1.)

	exp_obstacles = rospy.get_param("/coverage_obstacles", dict())
	for obs_name, obs in exp_obstacles.items():
		if len(obs) < 3:
			rospy.logerr("Invalid obstacle polygon!")
			sys.exit(1)

		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)
		exp_obstacles[obs_name] = sg.Polygon(obs).buffer(1.)

	partitions = dict()
	id_limit = 2

	for i in range(1, id_limit + 1):
		try:
			metric_partition_service = "/Agent{}/metric_partition".format(i)
			rospy.wait_for_service(metric_partition_service, 10)

			partition_client = rospy.ServiceProxy(metric_partition_service, GetMetricPartition)
			partition_response = partition_client(i)
			print("Agent {} returned.".format(i))
			partitions[i] = partition_response.partition
		except Exception as e:
			print(traceback.format_exc())
			# break

	figure = plt.figure()
	ax = figure.add_subplot(1, 1, 1)

	ax.clear()
	ax.set_aspect("equal")
	ax.set_title("Metric Graph Partition")
	ax.set_xlim(limits[0] - 5., limits[2] + 5.)
	ax.set_ylim(limits[1] - 5., limits[3] + 5.)

	if region_patch is not None:
		ax.add_patch(region_patch)

	for _, obs_patch in obstacle_patches.items():
		ax.add_patch(obs_patch)

	for i, partition in partitions.items():
		xs, ys = [], []

		for p in partition:
			xs.append(p.x)
			ys.append(p.y)

		ax.scatter(xs, ys, s=1., color=__COLORS[i])

	plt.show()