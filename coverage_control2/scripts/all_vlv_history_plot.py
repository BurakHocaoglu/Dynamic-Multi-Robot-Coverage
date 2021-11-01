# Run this with conda environment created with Python 3 and scikit-geometry

import sys
import time
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

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

history = None
limits = None
stop = False
region_patch = None
history_index = -1
history_thread = None
obstacle_patches = dict()

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

def animate_history(i, ax, lims, hist):
	try:
		ax.clear()
		ax.set_aspect("equal")
		ax.set_title("History")
		ax.set_xlim(lims[0] - 5., lims[2] + 5.)
		ax.set_ylim(lims[1] - 5., lims[3] + 5.)

		if globals()["region_patch"] is not None:
			ax.add_patch(globals()["region_patch"])

		for _, obs_patch in globals()["obstacle_patches"].items():
			ax.add_patch(obs_patch)

		if hist is not None:
			for aid, hist_piece in hist.items():
				i = min(len(hist_piece["position"]) - 1, globals()["history_index"])
				pos = hist_piece["position"][i]
				poly = hist_piece["polygon"][i]
				holes = hist_piece["holes"][i]
				robot_color = globals()["__COLORS"][int(aid)]
				ax.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
				ax.add_patch(plt.Polygon(poly, fill=True, color=robot_color, alpha=0.1, zorder=2))

				for h in holes:
					ax.add_patch(plt.Polygon(h, fill=True, color=(0., 0., 0.), alpha=0.5, zorder=1))

				skeleton = get_skeleton(poly, holes)
				for h in skeleton.halfedges:
					if h.is_bisector:
						p1 = h.vertex.point
						p2 = h.opposite.vertex.point
						plt.plot([p1.x(), p2.x()], [p1.y(), p2.y()], 'r-', lw=1)

				for v in skeleton.vertices:
					plt.gcf().gca().add_artist(plt.Circle((v.point.x(), v.point.y()), 
														   v.time, color='blue', fill=False))

				mg_in, mg_b = get_metric_graph(np.array(poly, dtype=float), 
												[np.array(h, dtype=float) for h in holes])

				if len(mg_in) > 0:
					mg_in_xs, mg_in_ys = zip(*mg_in)
					ax.scatter(mg_in_xs, mg_in_ys, s=0.5, color=robot_color)

				if len(mg_b) > 0:
					mg_b_xs, mg_b_ys = zip(*mg_b)
					ax.scatter(mg_b_xs, mg_b_ys, s=0.5, color=robot_color)

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

	exp_region = [[80., 80.], [80., -80.], [-80., -80.], [-80., 80.]]
	# exp_region = [[80., 100.], [80., -40.], [-80., -40.], [-80., 100.]]
	# exp_region = [[-100., 90.], [-100., -70.], [-20., -70.], [-20., 90.]]

	xcoords, ycoords = zip(*exp_region)
	xmin, xmax = min(xcoords), max(xcoords)
	ymin, ymax = min(ycoords), max(ycoords)
	limits = (xmin, ymin, xmax, ymax)
	region_patch = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))

	exp_obstacles = dict()
	exp_obstacles = {
		# "obs1": [[-60., 60.], [-20., 60.], [-20., 20.], [-60., 20.]],
	 #    "obs2": [[20., 60.], [60., 60.], [60., 20.], [20., 20.]],
		# "obs3": [[20., -20.], [60., -20.], [60., -60.], [20., -60.]],
		# "obs4": [[-60., -20.], [-20., -20.], [-20., -60.], [-60., -60.]],
		# "obs5": [[-10., 10.], [10., 10.], [10., -10.], [-10., -10.]],
	}
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
	for obs_name, obs in exp_obstacles.items():
		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)

	with open(sys.argv[1], "r") as H:
		history = json.load(H)

	if history is None:
		print("Could not load history content!")
		sys.exit(1)

	history_thread = threading.Thread(name="history_iterator", target=history_iterator, args=(history, ))

	figure = plt.figure()
	hist_ax = figure.add_subplot(1, 1, 1)
	ani_func = animation.FuncAnimation(figure, 
									   partial(animate_history, 
									   		   ax=hist_ax, 
									   		   lims=limits, 
									   		   hist=history), 
									   interval=100)

	time.sleep(1.)
	history_thread.start()

	plt.show(block=True)

	stop = True
	history_thread.join()
