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
	T = [(-75., 30.), (30., 70.), (65., -30.), (65., -70.), (-50., -75.)]
	holes = []

	PW = [(-60., 67.), (60., 67.), (10., -18.), (9., -73.), (-18., -33.)]
	GW = [(-44.1, 51.9), (44.4, 41.9), (30.5, -2.1), (36.8, -57.1), (-46.1, -30.3)]

	dV = [(-40., 70.), (70., 50.), (40., 0.), (40., -60.), (-70., -40.)]

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
							  [0., 40.], [0., 80.]], fill=False, color=COLORS[1], linewidth=5., 
							  alpha=0.7))

	ax.add_patch(plt.Polygon([[0., 80.], [0., 40.], [80., 0.], 
							  [80., 80.]], fill=False, color=COLORS[2], linewidth=5., 
							  alpha=0.7))

	ax.add_patch(plt.Polygon([[80., 0.], [0., 40.], [-23.3, 16.7], 
							  [8.2, -46.5], [80., -46.5]], fill=False, color=COLORS[3], linewidth=5., 
							  alpha=0.7))

	ax.add_patch(plt.Polygon([[80., -46.5], [8.2, -46.5], [-20., -80.], 
							  [80., -80.]], fill=False, color=COLORS[4], linewidth=5., 
							  alpha=0.7))

	ax.add_patch(plt.Polygon([[-20., -80.], [8.2, -46.5], [-23.3, 16.7], 
							  [-80., -6.], [-80., -80.]], fill=False, color=COLORS[5], linewidth=5., 
							  alpha=0.7))

	for i in range(1, len(P) + 1):
		p_i, q_i = P[i - 1], G[i - 1]
		v_i = np.array(q_i) - np.array(p_i)

		pw_i, gw_i = PW[i - 1], GW[i - 1]
		dV_i = dV[i - 1]

		# ax.add_artist(plt.Circle(q_i, 2., color=COLORS[0]))
		# ax.arrow(p_i[0], p_i[1], v_i[0] * 0.75, v_i[1] * 0.75, width=1.)
		ax.text(pw_i[0], pw_i[1], "${\\bf p}_{" + str(i) + "}$", color=COLORS[0], fontsize="large")
		# ax.text(gw_i[0], gw_i[1], "${\\bf C}_{V_{" + str(i) + "}}$", color=COLORS[0], fontsize="large")
		# ax.text(dV_i[0], dV_i[1], "$\\partial V_{" + str(i) + "}$", color=COLORS[0], fontsize="large")

	for i in range(len(T)):
		# ax.text(T[i][0], T[i][1], "$V_{" + str(i + 1) + "}$", color=COLORS[0], fontsize="x-large")
		ax.text(T[i][0], T[i][1], "$\\partial V_{" + str(i + 1) + "}$", color=COLORS[0], fontsize="large")

	plt.savefig("boundaries.png", bbox_inches="tight")

	plt.show()