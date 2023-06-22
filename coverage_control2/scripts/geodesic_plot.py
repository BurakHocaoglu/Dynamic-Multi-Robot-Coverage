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

if __name__ == "__main__":
	# region = np.array([[-30., 50.], [-50., 40.], [-50., -5.], [15., -5.], 
	# 					[-15., 10.], [-40., 10.], [-35., 20.], [25., 20.], 
	# 					[25., 30.], [-40., 30.], [-20., 40.], [25., 40.], 
	# 					[25., 50.]])

	region = np.array([[50., 110.], [-50., 110.], [-50., -20.], [50., -20.], 
						[50., 50.], [-40., 50.], [-40., 100.], [50., 100.]])

	# P = [(-60., 60.), (60., 60.), (20., -20.), (20., -73.), (-20., -40.)]
	p_i = (25., -15.)
	g_i = (-45., 45.)

	holes = {
		"obs0": [[-40., 40.], [40., 40.], [40., -10.], [-40., -10.]],
	}

	plt.rcParams.update({"text.usetex": True})

	geodesic_lloyd_figure = plt.figure(num=5)
	ax = geodesic_lloyd_figure.add_subplot(1, 1, 1)

	xs, ys = zip(*region)
	xmin, xmax = min(xs), max(xs)
	ymin, ymax = min(ys), max(ys)

	ax.clear()
	ax.set_aspect("equal")
	ax.set_xlim(xmin - 0.5, xmax + 0.5)
	ax.set_ylim(ymin - 0.5, ymax + 0.5)
	ax.set_axis_off()

	ax.add_patch(plt.Polygon(region, fill=False, color=COLORS[0]))

	for _, hole in holes.items():
		ax.add_patch(plt.Polygon(hole, fill=True, color=COLORS[0], alpha=0.5))

	ax.add_artist(plt.Circle(p_i, 2., color=COLORS[3]))
	ax.add_artist(plt.Circle(g_i, 2., color=COLORS[3]))
	ax.add_artist(plt.Circle((-40., -10.), 2., color=COLORS[3]))

	ax.text(28., -15, "${\\bf p}_{i}$", color=COLORS[0], fontsize="large")
	ax.text(-42., 43, "${\\bf C}_{V_{i}}$", color=COLORS[0], fontsize="large")
	ax.text(-46., -16., "${\\bf h}_{{\\bf C}_{V_{i}},{\\bf p}_{i}}$", color=COLORS[0], fontsize="large")

	ax.plot([25., -40.], [-15., -10.], "--", color=COLORS[2], linewidth=1.5)
	ax.plot([-40., -45.], [-10., 45.], "--", color=COLORS[2], linewidth=1.5)
	ax.plot([25., -45.], [-15., 45.], "--", color=COLORS[1], linewidth=1.5)

	v_i = np.array((-40., -10.)) - np.array(p_i)

	ax.arrow(p_i[0], p_i[1], v_i[0] * 0.45, v_i[1] * 0.45, width=1.)

	plt.savefig("geodesic_lloyd.png", bbox_inches="tight")

	plt.show()