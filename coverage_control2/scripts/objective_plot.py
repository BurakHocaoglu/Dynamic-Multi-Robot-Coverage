# -*- coding: utf-8 -*-

import sys
import time
import copy
import json
import signal
import threading
import traceback
import numpy as np
import skgeom as sg
import shapely.geometry as shgeom

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from functools import partial
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

COLORS = [(0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.), (1., 0., 1.), (0.5, 0.5, 0.5)]

yellow = (1., 1., 0.)

if __name__ == "__main__":
	region = np.array([[80., 80.], [80., -80.], [-80., -80.], [-80., 80.]])
	P = [(-60., 60.), (60., 60.), (20., -20.), (20., -73.), (-20., -40.)]
	G = [(-44.1, 44.9), (44.4, 48.9), (30.5, -9.1), (36.8, -64.1), (-41.1, -37.3)]
	T = [(-70., 30.), (30., 70.), (70., -30.), (70., -70.), (-50., -70.)]
	holes = []

	PW = [(-60., 67.), (60., 67.), (10., -18.), (17., -73.), (-18., -33.)]
	GW = [(-44.1, 51.9), (44.4, 41.9), (30.5, -2.1), (36.8, -57.1), (-46.1, -30.3)]

	plt.rcParams.update({"text.usetex": True})

	objective_figure = plt.figure(num=5)
	ax = objective_figure.add_subplot(1, 1, 1)

	xs, ys = zip(*region)
	xmin, xmax = min(xs), max(xs)
	ymin, ymax = min(ys), max(ys)

	ax.clear()
	ax.set_aspect("equal")
	ax.set_xlim(xmin - 0.5, xmax + 0.5)
	ax.set_ylim(ymin - 0.5, ymax + 0.5)
	ax.set_axis_off()

	ax.add_patch(plt.Polygon(region, fill=False, color=COLORS[0]))

	for hole in holes:
		ax.add_patch(plt.Polygon(hole, fill=True, color=COLORS[0], alpha=0.5))

	i = 1
	for p_i in P:
		ax.add_artist(plt.Circle(p_i, 2., color=COLORS[i]))
		i += 1

	ax.add_patch(plt.Polygon([[-80., 80.], [-80., -6.], [-23.3, 16.7], 
							  [0., 40.], [0., 80.]], fill=True, color=COLORS[1], alpha=0.2))
	ax.add_patch(plt.Polygon([[0., 80.], [0., 40.], [80., 0.], 
							  [80., 80.]], fill=True, color=COLORS[2], alpha=0.2))
	ax.add_patch(plt.Polygon([[80., 0.], [0., 40.], [-23.3, 16.7], 
							  [8.2, -46.5], [80., -46.5]], fill=True, color=COLORS[3], alpha=0.2))
	ax.add_patch(plt.Polygon([[80., -46.5], [8.2, -46.5], [-20., -80.], 
							  [80., -80.]], fill=True, color=COLORS[4], alpha=0.2))
	ax.add_patch(plt.Polygon([[-20., -80.], [8.2, -46.5], [-23.3, 16.7], 
							  [-80., -6.], [-80., -80.]], fill=True, color=COLORS[5], alpha=0.2))

	for i in range(1, len(P) + 1):
		p_i, q_i = P[i - 1], G[i - 1]
		v_i = np.array(q_i) - np.array(p_i)

		pw_i, gw_i = PW[i - 1], GW[i - 1]

		ax.add_artist(plt.Circle(q_i, 2., color=COLORS[0]))
		ax.arrow(p_i[0], p_i[1], v_i[0] * 0.75, v_i[1] * 0.75, width=1.)
		ax.text(pw_i[0], pw_i[1], "${\\bf p}_{" + str(i) + "}$", color=COLORS[0], fontsize="large")
		ax.text(gw_i[0], gw_i[1], "${\\bf C}_{V_{" + str(i) + "}}$", color=COLORS[0], fontsize="large")

	for i in range(len(T)):
		ax.text(T[i][0], T[i][1], "$V_{" + str(i + 1) + "}$", color=COLORS[0], fontsize="x-large")

	plt.savefig("total_placement_control.png", bbox_inches="tight")

	# -------------------------------------------------------------------------------------------------

	A = [5436.7, 4800, 6067.4, 2867.8, 6428.1]
	N = [(1, 2, 4), (0, 2), (0, 1, 3, 4), (2, 4), (0, 2, 3)]

	geodesic_placement_control = plt.figure(num=6)
	ax_control = geodesic_placement_control.add_subplot(1, 1, 1)

	ax_control.clear()
	ax_control.set_aspect("equal")
	ax_control.set_xlim(xmin - 0.5, xmax + 0.5)
	ax_control.set_ylim(ymin - 0.5, ymax + 0.5)
	ax_control.set_axis_off()

	ax_control.add_patch(plt.Polygon(region, fill=False, color=COLORS[0]))

	for hole in holes:
		ax_control.add_patch(plt.Polygon(hole, fill=True, color=COLORS[0], alpha=0.5))

	i = 1
	for p_i in P:
		ax_control.add_artist(plt.Circle(p_i, 2., color=COLORS[i]))
		i += 1

	ax_control.add_patch(plt.Polygon([[-80., 80.], [-80., -6.], [-23.3, 16.7], 
							  		  [0., 40.], [0., 80.]], fill=True, color=COLORS[1], alpha=0.2))
	ax_control.add_patch(plt.Polygon([[0., 80.], [0., 40.], [80., 0.], 
							  		  [80., 80.]], fill=True, color=COLORS[2], alpha=0.2))
	ax_control.add_patch(plt.Polygon([[80., 0.], [0., 40.], [-23.3, 16.7], 
							  		  [8.2, -46.5], [80., -46.5]], fill=True, color=COLORS[3], alpha=0.2))
	ax_control.add_patch(plt.Polygon([[80., -46.5], [8.2, -46.5], [-20., -80.], 
							  		  [80., -80.]], fill=True, color=COLORS[4], alpha=0.2))
	ax_control.add_patch(plt.Polygon([[-20., -80.], [8.2, -46.5], [-23.3, 16.7], 
							  		  [-80., -6.], [-80., -80.]], fill=True, color=COLORS[5], alpha=0.2))

	for i in range(1, len(P) + 1):
		p_i, q_i = P[i - 1], G[i - 1]
		v_i = np.array(q_i) - np.array(p_i)

		pw_i, gw_i = PW[i - 1], GW[i - 1]

		ax_control.add_artist(plt.Circle(q_i, 2., color=yellow))
		ax_control.arrow(p_i[0], p_i[1], v_i[0] * 0.75, v_i[1] * 0.75, width=1., color=COLORS[0])
		ax_control.text(pw_i[0], pw_i[1], "${\\bf p}_{" + str(i) + "}$", color=COLORS[0], fontsize="large")
		ax_control.text(gw_i[0], gw_i[1], "${\\bf C}_{V_{" + str(i) + "}}$", color=COLORS[0], fontsize="large")

	done_set = set()

	for i in range(len(N)):
		for j in N[i]:
			if (i, j) not in done_set:
				done_set.add((i, j))
				done_set.add((j, i))

			else:
				continue

			mid = (np.array(P[i]) + np.array(P[j])) * 0.5
			ax_control.add_artist(plt.Circle(tuple(mid), 2., color=COLORS[0]))
			ax_control.text(mid[0] - 2., mid[1] - 7., 
							"${\\bf m}_{" + str(i + 1) + ", " + str(j + 1) + "}$", 
							color=COLORS[0], fontsize="large")

			# v1 = (mid - np.array(P[i])) * ((-1) ** int(A[i] > A[j]))
			# v2 = (mid - np.array(P[j])) * ((-1) ** int(A[i] > A[j]))

			v1 = (mid - np.array(P[i])) * (A[j] - A[i]) * 0.0002
			v2 = (mid - np.array(P[j])) * (A[i] - A[j]) * 0.0002

			# v1 /= np.linalg.norm(v1)
			# v2 /= np.linalg.norm(v2)

			ax_control.plot([P[i][0], mid[0]], [P[i][1], mid[1]], "--", color=COLORS[-1], alpha=0.5)
			ax_control.plot([P[j][0], mid[0]], [P[j][1], mid[1]], "--", color=COLORS[-1], alpha=0.5)

			ax_control.arrow(P[i][0], P[i][1], v1[0], v1[1], width=0.5, color=COLORS[-1])
			ax_control.arrow(P[j][0], P[j][1], v2[0], v2[1], width=0.5, color=COLORS[-1])

	# for i in range(len(P)):
	# 	for j in range(len(P)):
	# 		if i == j:
	# 			continue

	# 		if j in N[i] and i in N[j]:
	# 			mid = (np.array(P[i]) + np.array(P[j])) * 0.25

	# 			v1 = (mid - np.array(P[i])) * 0.5 * ((-1) ** int(A[i] > A[j]))
	# 			v2 = (mid - np.array(P[j])) * 0.5 * ((-1) ** int(A[i] > A[j]))

	# 			ax_control.plot([P[i][0], mid[0]], [P[i][1], mid[1]], "--", color=COLORS[-1], alpha=0.5)
	# 			ax_control.plot([P[j][0], mid[0]], [P[j][1], mid[1]], "--", color=COLORS[-1], alpha=0.5)

	# 			ax_control.arrow(P[i][0], P[i][1], v1[0], v1[1], width=1., color=COLORS[-1])
	# 			ax_control.arrow(P[j][0], P[j][1], v2[0], v2[1], width=1., color=COLORS[-1])

	plt.savefig("total_geodesic_placement_control.png", bbox_inches="tight")

	plt.show()