
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

def plot_region_and_agents(ax, p_i, p_j, Q, holes=[]):
	# Get bbox limits pf "Q"
	xs, ys = zip(*Q)
	xmin, xmax = min(xs), max(xs)
	ymin, ymax = min(ys), max(ys)

	# Set drawing properties of "ax"
	ax.clear()
	ax.set_aspect("equal")
	ax.set_xlim(xmin - 0.5, xmax + 0.5)
	ax.set_ylim(ymin - 0.5, ymax + 0.5)
	ax.set_axis_off()

	# Plot agents, "p_i" and "p_j"
	ax.add_artist(plt.Circle(tuple(p_i), 0.2, color=(0., 0., 1.)))
	ax.add_artist(plt.Circle(tuple(p_j), 0.2, color=(1., 0., 0.)))

	# Plot polygonal region, "Q"
	ax.add_patch(plt.Polygon(Q, fill=False, color=(0., 0., 0.)))

	for hole in holes:
		ax.add_patch(plt.Polygon(hole, fill=True, color=(0., 0., 0.), alpha=0.5))

if __name__ == "__main__":
	region = np.array([[8., 10.], [-10., 6.], [0., 2.], [-12., -6.], [4., -8.]])
	p_i = np.array([-2., -2.])
	p_j_1 = np.array([2., 0.])
	p_j_2 = np.array([6., 6.])

	# ---------------------------------------------------------------------------------
	# Disconnected subregions issue with Euclidean Voronoi diagram
	disconnection_figure = plt.figure(num=1)
	ax_disconnection = disconnection_figure.add_subplot(1, 1, 1)

	plot_region_and_agents(ax_disconnection, p_i, p_j_1, region)

	red_piece = np.array([[8., 10.], [-4.2, 7.3], [-1.9, 2.8], [0., 2.], 
						  [-1.1, 1.3], [3.5, -7.9], [4., -8.]])

	blue_piece_1 = np.array([[-10., 6.], [-4.2, 7.3], [-1.9, 2.8]])
	blue_piece_2 = np.array([[-1.1, 1.3], [-12., -6.], [3.5, -7.9]])

	ax_disconnection.add_patch(plt.Polygon(red_piece, fill=True, color=(1., 0., 0.), alpha=0.3))
	ax_disconnection.add_patch(plt.Polygon(blue_piece_1, fill=True, color=(1., 0., 1.), alpha=0.3))
	ax_disconnection.add_patch(plt.Polygon(blue_piece_2, fill=True, color=(0., 0., 1.), alpha=0.3))

	plt.savefig("disconnected.png", bbox_inches="tight")
	# ---------------------------------------------------------------------------------

	# ---------------------------------------------------------------------------------
	# Euclidean Voronoi diagram 2
	euclidean_voronoi_figure = plt.figure(num=2)
	ax_euclidean_voronoi = euclidean_voronoi_figure.add_subplot(1, 1, 1)

	plot_region_and_agents(ax_euclidean_voronoi, p_i, p_j_2, region)

	red_piece = np.array([[8., 10.], [-3.5, 7.5], [5.5, -1.5]])

	blue_piece = np.array([[-3.5, 7.5], [-10., 6.], [0., 2.], 
						   [-12., -6.], [4., -8.], [5.5, -1.5]])

	ax_euclidean_voronoi.add_patch(plt.Polygon(red_piece, fill=True, color=(1., 0., 0.), alpha=0.3))
	ax_euclidean_voronoi.add_patch(plt.Polygon(blue_piece, fill=True, color=(0., 0., 1.), alpha=0.3))

	test_point = np.array([-2., 6.])

	ax_euclidean_voronoi.add_artist(plt.Circle(tuple(test_point), 0.2, fill=True, color=(0., 0., 0.)))
	ax_euclidean_voronoi.plot([p_i[0], test_point[0]], [p_i[1], test_point[1]], 
								color=(1., 0., 0.), linewidth=2.)
	ax_euclidean_voronoi.plot([p_j_2[0], test_point[0]], [p_j_2[1], test_point[1]], 
								color=(0., 1., 0.), linewidth=2.)

	ax_euclidean_voronoi.text(2., 6.2, "d = 8", color=(0., 0., 0.))
	ax_euclidean_voronoi.text(-4.2, 2., "d = 8", color=(1., 0., 0.))

	plt.savefig("euclidean_voronoi.png", bbox_inches="tight")
	# ---------------------------------------------------------------------------------

	# ---------------------------------------------------------------------------------
	# Geodesic Voronoi diagram 1
	geodesic_voronoi_figure = plt.figure(num=3)
	ax_geodesic_voronoi = geodesic_voronoi_figure.add_subplot(1, 1, 1)

	plot_region_and_agents(ax_geodesic_voronoi, p_i, p_j_1, region)

	red_piece = np.array([[8., 10.], [-10., 6.], [0., 2.], [-1.1, 1.3], 
						  [3.5, -7.9], [4., -8.]])

	blue_piece = np.array([[-1.1, 1.3], [-12., -6.], [3.5, -7.9]])

	ax_geodesic_voronoi.add_patch(plt.Polygon(red_piece, fill=True, color=(1., 0., 0.), alpha=0.3))
	ax_geodesic_voronoi.add_patch(plt.Polygon(blue_piece, fill=True, color=(0., 0., 1.), alpha=0.3))

	plt.savefig("geodesic_voronoi_1.png", bbox_inches="tight")
	# ---------------------------------------------------------------------------------

	# ---------------------------------------------------------------------------------
	# Geodesic Voronoi diagram 2
	geodesic_voronoi_figure_2 = plt.figure(num=4)
	ax_geodesic_voronoi_2 = geodesic_voronoi_figure_2.add_subplot(1, 1, 1)

	plot_region_and_agents(ax_geodesic_voronoi_2, p_i, p_j_2, region)

	N = 100
	x_axis = np.linspace(-6.6, 0.7, N)
	y_axis = np.linspace(3.3, 6.8, N)
	X, Y = np.meshgrid(x_axis, y_axis)

	F = lambda q, p: np.sqrt((q - 6.) ** 2 + (p - 6.) ** 2) - np.sqrt(q ** 2 + (p - 2.) ** 2) - 2. * np.sqrt(5.)

	red_piece = [[5.5, -1.5], [8., 10.], [-3.5, 7.5]]
	blue_piece = [[5.5, -1.5], [4., -8.], [-12., -6.], [0., 2.], [-10., 6.]]

	shared_segment = []

	rows, cols = X.shape
	for i in range(rows):
		for j in range(cols):
			if np.absolute(F(X[i, j], Y[i, j])) <= 5e-3:
				shared_segment.append([X[i, j], Y[i, j]])

	red_piece.extend(shared_segment[::-1])
	blue_piece.extend(shared_segment[::-1])

	ax_geodesic_voronoi_2.add_patch(plt.Polygon(red_piece, fill=True, color=(1., 0., 0.), alpha=0.3))
	ax_geodesic_voronoi_2.add_patch(plt.Polygon(blue_piece, fill=True, color=(0., 0., 1.), alpha=0.3))

	test_point = np.array([-2., 5.])

	ax_geodesic_voronoi_2.add_artist(plt.Circle(tuple(test_point), 0.2, fill=True, color=(0., 0., 0.)))
	ax_geodesic_voronoi_2.plot([p_i[0], test_point[0]], [p_i[1], test_point[1]], 
								color=(1., 0., 0.), linewidth=2.)
	ax_geodesic_voronoi_2.plot([p_j_2[0], test_point[0]], [p_j_2[1], test_point[1]], 
								color=(0., 1., 0.), linewidth=2.)

	ax_geodesic_voronoi_2.plot([p_i[0], 0.], [p_i[1], 2.], color=(0., 1., 0.), linewidth=2.)
	ax_geodesic_voronoi_2.plot([0., test_point[0]], [2., test_point[1]], 
								color=(0., 1., 0.), linewidth=2.)

	ax_geodesic_voronoi_2.text(1., 6., "d = 8.1", color=(0., 0., 0.))
	ax_geodesic_voronoi_2.text(-0.2, 0., "d1 = 4.5", color=(0., 0., 0.))
	ax_geodesic_voronoi_2.text(-0.4, 3.2, "d2 = 3.6", color=(0., 0., 0.))
	ax_geodesic_voronoi_2.text(-4.2, 2., "d = 7", color=(1., 0., 0.))

	plt.savefig("geodesic_voronoi_2.png", bbox_inches="tight")
	# ---------------------------------------------------------------------------------

	# ---------------------------------------------------------------------------------
	# Geodesic path and distance visualization
	plt.rcParams.update({"text.usetex": True})

	region_2 = np.array([[-18., 16.], [-22., 8.], [-14., 6.], [-6., 14.], [-2., 8.], 
						[16., 10.], [10., 20.], [8., 12.], [2., 12.], [-4., 20.], [-14., 12.]])

	s_0 = np.array([-18., 14.])
	s_f = np.array([10., 16.])

	geodesic_path_figure = plt.figure(num=5)
	ax_geodesic_path = geodesic_path_figure.add_subplot(1, 1, 1)

	plot_region_and_agents(ax_geodesic_path, s_0, s_f, region_2)

	ax_geodesic_path.plot([s_0[0], s_f[0]], [s_0[1], s_f[1]], color=(1., 0., 0.), linewidth=2.)
	ax_geodesic_path.text(-5., 16., "Invalid", color=(0., 0., 0.))

	ax_geodesic_path.plot([s_0[0], -14.], [s_0[1], 12.], color=(0., 1., 0.), linewidth=2.)
	ax_geodesic_path.plot([-14., -6.], [12., 14.], color=(0., 1., 0.), linewidth=2.)
	ax_geodesic_path.plot([-6., 2.], [14., 12.], color=(0., 1., 0.), linewidth=2.)
	ax_geodesic_path.plot([2., 8.], [12., 12.], color=(0., 1., 0.), linewidth=2.)
	ax_geodesic_path.plot([8., s_f[0]], [12., s_f[1]], color=(0., 1., 0.), linewidth=2.)

	ax_geodesic_path.text(-18., 12.5, "${\\bf p}$", color=(0., 0., 0.))
	ax_geodesic_path.text(-14., 10.5, "${\\bf h}_{{\\bf q}, {\\bf p}}^{4}$", color=(0., 0., 0.))
	ax_geodesic_path.text(-7., 11., "${\\bf h}_{{\\bf q}, {\\bf p}}^{3}$", color=(0., 0., 0.))
	ax_geodesic_path.text(2., 10.3, "${\\bf h}_{{\\bf q}, {\\bf p}}^{2}$", color=(0., 0., 0.))
	ax_geodesic_path.text(8., 10.3, "${\\bf h}_{{\\bf q}, {\\bf p}}^{1}$", color=(0., 0., 0.))
	ax_geodesic_path.text(10., 14., "${\\bf q}$", color=(0., 0., 0.))

	plt.savefig("geodesic_path.png", bbox_inches="tight")
	# ---------------------------------------------------------------------------------

	plt.show()