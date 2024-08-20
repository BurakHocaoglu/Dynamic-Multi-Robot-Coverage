# Run this with conda environment created with Python 3 and scikit-geometry

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

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0,0.99), (0,0.99,0.99), 
			(0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5), (0.99,0.99,0)]

history = None
limits = None
stop = False
region_patch = None
region_patch2 = None
history_index = -1
history_thread = None
obstacle_patches = dict()
obstacle_patches2 = dict()

def get_skeleton(poly, holes=[]):
	sgp = sg.Polygon(poly)

	if sgp.orientation() == sg.Sign.CLOCKWISE:
		sgp.reverse_orientation()

	hole_list = []
	for h in holes:
		sgh = sg.Polygon(h)

		if sgh.orientation() != sg.Sign.CLOCKWISE:
			sgh.reverse_orientation()

		hole_list.append(sgh)

	full_poly = sg.PolygonWithHoles(sgp, hole_list)

	return sg.skeleton.create_interior_straight_skeleton(full_poly)

def is_state_valid(q, poly, holes=[]):
	if poly is None:
		return False

	if not poly.contains(q):
		return False

	for hole in holes:
		if hole.contains(q):
			return False

	return True

def get_metric_graph(poly, holes=[]):
	v_in, v_b = [], []

	resolution = 4.
	dense_resolution = resolution / 2.
	poly_shgeom = shgeom.Polygon(shell=poly, holes=[hole for hole in holes]).buffer(-resolution / 2.)
	xmin, ymin, xmax, ymax = poly_shgeom.bounds

	xidx = xmin
	while xidx < xmax:
		yidx = ymin
		while yidx < ymax:
			p = shgeom.Point(xidx, yidx)

			if poly_shgeom.contains(p):
				v_in.append((xidx, yidx))

			yidx += resolution
		xidx += resolution

	# for i in range(len(poly)):
	# 	j = (i + 1) % len(poly)
	# 	d = poly[j] - poly[i]
	# 	norm = np.linalg.norm(d)
	# 	nSteps = norm / dense_resolution
	# 	d *= dense_resolution / norm
	# 	n = np.array((-d[1], d[0]), dtype=float)

	# 	k = 0
	# 	while k <= nSteps + 1:
	# 		p_raw = poly[i] + k * d + n
	# 		p = shgeom.Point(p_raw[0], p_raw[1])

	# 		if poly_shgeom.contains(p):
	# 			v_b.append(p_raw)

	# 		k += resolution

	# for hole in holes:
	# 	for i in range(len(hole)):
	# 		j = (i + 1) % len(hole)
	# 		d = hole[j] - hole[i]
	# 		norm = np.linalg.norm(d)
	# 		nSteps = norm / dense_resolution
	# 		d *= dense_resolution / norm
	# 		n = np.array((-d[1], d[0]), dtype=float)

	# 		k = 0
	# 		while k <= nSteps + 1:
	# 			p_raw = hole[i] + k * d + n
	# 			p = shgeom.Point(p_raw[0], p_raw[1])

	# 			if poly_shgeom.contains(p):
	# 				v_b.append(p_raw)

	# 			k += dense_resolution

	return v_in, v_b

def compute_area_workload(poly, holes=[]):
	sh_poly = shgeom.Polygon(poly)
	outer_W = sh_poly.area

	for hole in holes:
		sh_hole = shgeom.Polygon(hole)
		outer_W -= sh_hole.area

	# assert (outer_W > 0.)

	return outer_W

def multivariate_gaussian(pos, mu, Sigma):
    n = mu.shape[0]
    Sigma_det = np.linalg.det(Sigma)
    Sigma_inv = np.linalg.inv(Sigma)
    N = np.sqrt((2. * np.pi) ** n * Sigma_det)
    fac = np.einsum('...k,kl,...l->...', pos - mu, Sigma_inv, pos - mu)

    return np.exp(- fac / 2.) / N

def animate_history(i, ax, lims, hist):
	try:
		ax.clear()
		ax.set_aspect("equal")
		ax.set_title("Dist. Cov. Experiment")
		ax.set_xlim(lims[0] - 5., lims[2] + 5.)
		ax.set_ylim(lims[1] - 5., lims[3] + 5.)

		if globals()["region_patch"] is not None:
			ax.add_patch(globals()["region_patch"])

		for _, obs_patch in globals()["obstacle_patches"].items():
			ax.add_patch(obs_patch)

		if hist is not None:
			workloads = []
			for aid, hist_piece in hist.items():
				i = min(len(hist_piece["position"]) - 1, globals()["history_index"])
				pos = hist_piece["position"][i]
				poly = hist_piece["polygon"][i]
				holes = hist_piece["holes"][i]
				robot_color = globals()["__COLORS"][int(aid)]
				ax.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
				ax.add_patch(plt.Polygon(poly, fill=True, color=robot_color, alpha=0.3, zorder=2))

				for h in holes:
					# ax.add_patch(plt.Polygon(h, fill=True, color=(0., 0., 0.), alpha=0.5, zorder=1))
					ax.add_patch(plt.Polygon(h, fill=True, color=(0., 0., 0.), alpha=1, zorder=1))

				workloads.append(compute_area_workload(poly, holes))

				skeleton = get_skeleton(poly, holes)
				for h in skeleton.halfedges:
					if h.is_bisector:
						p1 = h.vertex.point
						p2 = h.opposite.vertex.point
						plt.plot([p1.x(), p2.x()], [p1.y(), p2.y()], 'r-', lw=1)

				# for v in skeleton.vertices:
				# 	plt.gcf().gca().add_artist(plt.Circle((v.point.x(), v.point.y()), 
				# 										   v.time, color='blue', fill=False))

				# mg_in, mg_b = get_metric_graph(np.array(poly, dtype=float), 
				# 								[np.array(h, dtype=float) for h in holes])

				# if len(mg_in) > 0:
				# 	mg_in_xs, mg_in_ys = zip(*mg_in)
				# 	ax.scatter(mg_in_xs, mg_in_ys, s=0.5, color=robot_color)

				# if len(mg_b) > 0:
				# 	mg_b_xs, mg_b_ys = zip(*mg_b)
				# 	ax.scatter(mg_b_xs, mg_b_ys, s=0.5, color=robot_color)

			if len(workloads) > 0:
				std = np.std(workloads)
				print("Workload Std. - Var.: {} - {}".format(std, std ** 2))

	except Exception as e:
		# raise e
		print(traceback.format_exc())

def history_iterator(hist):
	while not globals()["stop"] or globals()["history_index"] < len(hist):
		time.sleep(0.5)
		globals()["history_index"] += 1

def get_total_path_traversed(pos_history):
	total = 0.
	for i in range(len(pos_history) - 1):
		total += np.linalg.norm(np.array(pos_history[i + 1]) - np.array(pos_history[i]))

	return total

def customSigintHandler(signum, frame):
	globals()["stop"] = True

	if globals()["history_thread"] is not None:
		globals()["history_thread"].join()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, customSigintHandler)

	with_skel = int(sys.argv[2]) == 1

	with open(sys.argv[1], "r") as H:
		history = json.load(H)

	if history is None:
		print("Could not load history content!")
		sys.exit(1)

	region_patch = plt.Polygon(history["coverage_region"], fill=False, color=(0., 0., 0.))
	region_patch2 = plt.Polygon(history["coverage_region"], fill=False, color=(0., 0., 0.))

	xcoords, ycoords = zip(*(history["coverage_region"]))
	xmin, xmax = min(xcoords), max(xcoords)
	ymin, ymax = min(ycoords), max(ycoords)
	limits = (xmin, ymin, xmax, ymax)

	for obs_name, obs in history["coverage_obstacles"].items():
		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)
		obstacle_patches2[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)

	print([len(history[str(j + 1)]["position"]) for j in range(history["agent_count"])])

	# -----------------------------------------------------------------------------------------------
	print("******************************* START *******************************")

	figure1 = plt.figure(num=1)
	ax1 = figure1.add_subplot(1, 1, 1)

	N = 600
	X = np.linspace(xmin - 5., xmax + 5., N)
	Y = np.linspace(ymin - 5., ymax + 5., N)
	X, Y = np.meshgrid(X, Y)

	Sigma = np.array([[200. , 0.], 
	                  [0.,  200.]])

	mvpos = np.empty(X.shape + (2,))
	mvpos[:, :, 0] = X
	mvpos[:, :, 1] = Y

	ax1.clear()
	ax1.set_aspect("equal")
	ax1.set_xlim(limits[0] - 0.5, limits[2] + 0.5)
	ax1.set_ylim(limits[1] - 0.5, limits[3] + 0.5)
	ax1.set_axis_off()

	if region_patch is not None:
		ax1.add_patch(region_patch2)

	for _, obs_patch in obstacle_patches2.items():
		ax1.add_patch(obs_patch)

	if history is not None:
		workloads = []
		for aid in range(history["agent_count"]):
			pos = history[str(aid + 1)]["position"][10]
			poly = history[str(aid + 1)]["polygon"][10]
			holes = history[str(aid + 1)]["holes"][10]
			robot_color = __COLORS[aid + 1]
			ax1.add_artist(plt.Circle(tuple(pos), 2., color=robot_color))
			ax1.add_patch(plt.Polygon(poly, fill=True, color=robot_color, alpha=0.2, zorder=2))

			for h in holes:
				ax1.add_patch(plt.Polygon(h, fill=True, color=(0., 0., 0.), alpha=1, zorder=1))

			workloads.append(compute_area_workload(poly, holes))

			# Z = multivariate_gaussian(mvpos, np.array(pos), Sigma) * 1e5
			# cset = ax1.contourf(X, Y, Z, levels=np.linspace(10., 100., 11), 
			# 				   cmap=cm.coolwarm, alpha=0.7)

			# skeleton = get_skeleton(poly, holes)
			# for h in skeleton.halfedges:
			# 	if h.is_bisector:
			# 		p1 = h.vertex.point
			# 		p2 = h.opposite.vertex.point
			# 		plt.plot([p1.x(), p2.x()], [p1.y(), p2.y()], 'r-', lw=1)

			# for v in skeleton.vertices:
			# 	plt.gcf().gca().add_artist(plt.Circle((v.point.x(), v.point.y()), 
			# 										   v.time, color='blue', fill=False))

			# mg_in, mg_b = get_metric_graph(np.array(poly, dtype=float), 
			# 								[np.array(h, dtype=float) for h in holes])

			# if len(mg_in) > 0:
			# 	mg_in_xs, mg_in_ys = zip(*mg_in)
			# 	ax.scatter(mg_in_xs, mg_in_ys, s=0.5, color=robot_color)

			# if len(mg_b) > 0:
			# 	mg_b_xs, mg_b_ys = zip(*mg_b)
			# 	ax.scatter(mg_b_xs, mg_b_ys, s=0.5, color=robot_color)

		if len(workloads) > 0:
			std = np.std(workloads)
			print("Workload Std. - Var.: {} - {}".format(std, std ** 2))

	# plt.savefig('F_h1.png', bbox_inches='tight')

	# -----------------------------------------------------------------------------------------------
	print("******************************* FINAL *******************************")

	figure2 = plt.figure(num=2)
	ax2 = figure2.add_subplot(1, 1, 1)

	# N = 600
	# X = np.linspace(xmin - 5., xmax + 5., N)
	# Y = np.linspace(ymin - 5., ymax + 5., N)
	# X, Y = np.meshgrid(X, Y)

	# Sigma = np.array([[200. , 0.], 
	#                   [0.,  200.]])

	# mvpos = np.empty(X.shape + (2,))
	# mvpos[:, :, 0] = X
	# mvpos[:, :, 1] = Y

	ax2.clear()
	ax2.set_aspect("equal")
	ax2.set_xlim(limits[0] - 0.5, limits[2] + 0.5)
	ax2.set_ylim(limits[1] - 0.5, limits[3] + 0.5)
	ax2.set_axis_off()

	if region_patch is not None:
		ax2.add_patch(region_patch)

	for _, obs_patch in obstacle_patches.items():
		ax2.add_patch(obs_patch)

	if history is not None:
		workloads = []
		for aid in range(history["agent_count"]):
			pos = history[str(aid + 1)]["position"][-1]
			poly = history[str(aid + 1)]["polygon"][-1]
			holes = history[str(aid + 1)]["holes"][-1]
			robot_color = __COLORS[aid + 1]
			ax2.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
			ax2.add_patch(plt.Polygon(poly, fill=True, color=robot_color, alpha=0.2, zorder=2))

			x_hist, y_hist = zip(*(history[str(aid + 1)]["position"]))
			ax2.plot(x_hist, y_hist, color=robot_color, linewidth=2., alpha=0.5)

			for h in holes:
				ax2.add_patch(plt.Polygon(h, fill=True, color=(0., 0., 0.), alpha=1, zorder=1))

			workloads.append(compute_area_workload(poly, holes))
			print(f"Agent {aid + 1} has traversed {get_total_path_traversed(history[str(aid + 1)]['position'])} units (m).")

			# Z = multivariate_gaussian(mvpos, np.array(pos), Sigma) * 1e5
			# cset = ax2.contourf(X, Y, Z, levels=np.linspace(10., 100., 11), 
			# 				   cmap=cm.coolwarm, alpha=0.7)

			if with_skel:
				skeleton = get_skeleton(poly, holes)
				for h in skeleton.halfedges:
					if h.is_bisector:
						p1 = h.vertex.point
						p2 = h.opposite.vertex.point
						plt.plot([p1.x(), p2.x()], [p1.y(), p2.y()], color=robot_color, lw=1)

			# for v in skeleton.vertices:
			# 	plt.gcf().gca().add_artist(plt.Circle((v.point.x(), v.point.y()), 
			# 										   v.time, color='blue', fill=False))

			# mg_in, mg_b = get_metric_graph(np.array(poly, dtype=float), 
			# 								[np.array(h, dtype=float) for h in holes])

			# if len(mg_in) > 0:
			# 	mg_in_xs, mg_in_ys = zip(*mg_in)
			# 	ax.scatter(mg_in_xs, mg_in_ys, s=0.5, color=robot_color)

			# if len(mg_b) > 0:
			# 	mg_b_xs, mg_b_ys = zip(*mg_b)
			# 	ax.scatter(mg_b_xs, mg_b_ys, s=0.5, color=robot_color)

		if len(workloads) > 0:
			print("Workloads: {}".format(workloads))
			std = np.std(workloads)
			print("Workload Std. - Var.: {} - {}".format(std, std ** 2))

	plt.savefig('F_h2.png', bbox_inches='tight')

	plt.show()