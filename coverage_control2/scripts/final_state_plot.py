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

def customSigintHandler(signum, frame):
	globals()["stop"] = True

	if globals()["history_thread"] is not None:
		globals()["history_thread"].join()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, customSigintHandler)

	with_skel = int(sys.argv[2]) == 1

	exp_region = [[80., 80.], [80., -80.], [-80., -80.], [-80., 80.]]
	# exp_region = [[80., 100.], [80., -40.], [-80., -40.], [-80., 100.]]
	# exp_region = [[-20., 50.], [-50., 20.], [-40., -20.], [10., -50.], [40., 10.], [30., 40.]]
	# exp_region = [[30., -20.], [30., -40.], [10., -50.], [40., -60.], [20., -110.], [40., -90.], 
	# 				[90., -100.], [70., -80.], [80., -50.], [70., -10.], [50., -30.]]

	# exp_region = [[-100., 90.], [-100., -70.], [-20., -70.], [-20., 90.]]
	# exp_region = [[-100., 90.], [-100., 70.], [-70., 70.], [-70., 64.], 
	# 				[-100., 64.], [-100., 26.], [-72., 26.], [-72., 38.], 
	# 				[-66., 38.], [-66., 22.], [-100., 22.], [-100., -70.], 
	# 				[-80., -70.], [-80., -22.], [-74., -22.], [-74., -48.], 
	# 				[-54., -48.], [-54., -54.], [-74., -54.], [-74., -70.], 
	# 				[-20., -70.], [-20., 10.], [-60., 10.], [-60., 14.], 
	# 				[-20., 14.], [-20., 52.], [-50., 52.], [-50., 56.], 
	# 				[-20., 56.], [-20., 90.], [-56., 90.], [-56., 74.], 
	# 				[-60., 74.], [-60., 90.]]

	exp_region = [[-80., 130.], [-80., 10.], [20., 10.], [20., 80.], 
				[-70., 80.], [-70., 120.], [20., 120.], [20., 130.]]

	# exp_region = [[-90., 120.], [-90., 10.], [-10., 10.], [-10., 20.], 
	# 			[-80., 20.], [-80., 40.], [-10., 40.], [-10., 60.], 
	# 			[-80., 50.], [-80., 70.], [-30., 60.], [-20., 80.], 
	# 			[-80., 80.], [-80., 90.], [-10., 90.], [-10., 100.], 
	# 			[-80., 100.], [-80., 110.], [-10., 110.], [-10., 120.]]

	# exp_region = np.array([[80., 140.], [120., 100.], [80., 100.], [20., 140.], 
	# 						[40., 60.], [-40., 40.], [80., -20.], [80., 80.], 
	# 						[160., -20.], [140., 80.], [300., 20.], [300., 120.], 
	# 						[260., 60.], [180., 80.], [160., 180.], [320., 160.], 
	# 						[140., 220.], [140., 120.]])

	# exp_region = [[-140., 120.], [-200., 20.], [-200., -40.], [-160., -40.], [-40., 20.], [-60., 100.]]

	# exp_region = np.array([[60., 60.], [-60., 23.], [20., -60.]])

	# exp_region = np.array([[-90., 120.], [-90., 10.], [-10., 10.], [-10., 20.], 
	# 						[-80., 20.], [-80., 40.], [-10., 40.], [-10., 60.], 
	# 						[-80., 50.], [-80., 70.], [-30., 60.], [-20., 80.], 
	# 						[-80., 80.], [-80., 90.], [-10., 90.], [-10., 100.], 
	# 						[-80., 100.], [-80., 110.], [-10., 110.], [-10., 120.]])

	# exp_region = np.array([[120., 140.], [50., 140.], [50., 90.], [115., 90.],
	# 						[115., 130.], [60., 130.], [60., 100.], [105., 100.],
	# 						[105., 120.], [70., 120.], [70., 115.], [100., 115.],
	# 						[100., 105.], [65., 105.], [65., 125.], [110., 125.],
	# 						[110., 95.], [55., 95.], [55., 135.], [120., 135.]])

	xcoords, ycoords = zip(*exp_region)
	xmin, xmax = min(xcoords), max(xcoords)
	ymin, ymax = min(ycoords), max(ycoords)
	limits = (xmin, ymin, xmax, ymax)

	region_patch = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))
	region_patch2 = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))

	exp_obstacles = dict()
	# exp_obstacles = {
	# 	"obs1": [[-60., 60.], [-20., 60.], [-20., 20.], [-60., 20.]],
	# 	"obs2": [[20., 60.], [60., 60.], [60., 20.], [20., 20.]],
	# 	"obs3": [[20., -20.], [60., -20.], [60., -60.], [20., -60.]],
	# 	"obs4": [[-60., -20.], [-20., -20.], [-20., -60.], [-60., -60.]],
	# 	"obs5": [[-10., 10.], [10., 10.], [10., -10.], [-10., -10.]],
	# }
	# exp_obstacles = {
	# 	"obs1": [[-60., 87.], [-56., 87.], [-56., 74.], [-60., 74.]],
	# 	"obs2": [[-97., 70.], [-70., 70.], [-70., 64.], [-97., 64.]],
	# 	"obs3": [[-50., 56.], [-22., 56.], [-22., 53.], [-50., 53.]],
	# 	"obs4": [[-98., 26.], [-72., 26.], [-72., 37.], [-66., 37.], [-66., 23.], [-98., 23.]],
	# 	"obs5": [[-60., 14.], [-24., 14.], [-24., 9.], [-60., 9.]],
	# 	"obs6": [[-52., -14.], [-28., -14.], [-28., -30.], [-32., -30.], [-32., -17.], [-52., -17.]],
	# 	"obs7": [[-80., -22.], [-74., -22.], [-74., -48.], [-54., -48.], 
	# 			 [-54., -54.], [-74., -54.], [-74., -68.], [-80., -68.]],
	# }
	exp_obstacles = {
		# "obs1": [[60., -60.], [-60., -60.], [-60., 60.], [60., 60.]],
		"obs2": [[-70., 70.], [10., 70.], [10., 20.], [-70., 20.]],
		# "obs": [[60., -60.], [-60., -60.], [-60., 60.], [60., 60.]],
	}
	for obs_name, obs in exp_obstacles.items():
		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)
		obstacle_patches2[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)

	with open(sys.argv[1], "r") as H:
		history = json.load(H)

	if history is None:
		print("Could not load history content!")
		sys.exit(1)

	print("Length: {}".format([len(hist["position"]) for _, hist in history.items()]))

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
		for aid in range(len(history)):
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

	plt.savefig('F_h1.png', bbox_inches='tight')
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
		for aid in range(len(history)):
			pos = history[str(aid + 1)]["position"][-1]
			poly = history[str(aid + 1)]["polygon"][-1]
			holes = history[str(aid + 1)]["holes"][-1]
			robot_color = __COLORS[aid + 1]
			ax2.add_artist(plt.Circle(tuple(pos), 2., color=robot_color))
			ax2.add_patch(plt.Polygon(poly, fill=True, color=robot_color, alpha=0.2, zorder=2))

			x_hist, y_hist = zip(*(history[str(aid + 1)]["position"]))
			ax2.plot(x_hist, y_hist, color=robot_color, linewidth=2.)

			for h in holes:
				ax2.add_patch(plt.Polygon(h, fill=True, color=(0., 0., 0.), alpha=1, zorder=1))

			workloads.append(compute_area_workload(poly, holes))

			# Z = multivariate_gaussian(mvpos, np.array(pos), Sigma) * 1e5
			# cset = ax2.contourf(X, Y, Z, levels=np.linspace(10., 100., 11), 
			# 				   cmap=cm.coolwarm, alpha=0.7)

			if with_skel:
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
			print("Workloads: {}".format(workloads))
			std = np.std(workloads)
			print("Workload Std. - Var.: {} - {}".format(std, std ** 2))

	plt.savefig('F_h2.png', bbox_inches='tight')

	plt.show()