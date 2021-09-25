# Run this with conda environment created with Python 3 and scikit-geometry

import sys
import time
import json
import signal
import threading
import traceback
import numpy as np
import skgeom as sg

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

def get_skeleton(poly):
	sgp = sg.Polygon(poly)

	if sgp.orientation() == sg.Sign.CLOCKWISE:
		sgp.reverse_orientation()

	return sg.skeleton.create_interior_straight_skeleton(sgp)

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
				robot_color = globals()["__COLORS"][int(aid)]
				ax.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
				ax.add_patch(plt.Polygon(poly, fill=True, color=robot_color, alpha=0.3))

				skeleton = get_skeleton(poly)
				for h in skeleton.halfedges:
					if h.is_bisector:
						p1 = h.vertex.point
						p2 = h.opposite.vertex.point
						plt.plot([p1.x(), p2.x()], [p1.y(), p2.y()], 'r-', lw=1)
	except Exception as e:
		# raise e
		print(traceback.format_exc())

def history_iterator(hist):
	while not globals()["stop"] or globals()["history_index"] < len(hist):
		time.sleep(0.2)
		globals()["history_index"] += 1

def customSigintHandler(signum, frame):
	globals()["stop"] = True

	if globals()["history_thread"] is not None:
		globals()["history_thread"].join()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, customSigintHandler)

	# exp_region = [[80., 80.], [80., -80.], [-80., -80.], [-80., 80.]]
	exp_region = [[80., 100.], [80., -40.], [-80., -40.], [-80., 100.]]

	xcoords, ycoords = zip(*exp_region)
	xmin, xmax = min(xcoords), max(xcoords)
	ymin, ymax = min(ycoords), max(ycoords)
	limits = (xmin, ymin, xmax, ymax)
	region_patch = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))

	exp_obstacles = dict()
	exp_obstacles = {
		# "obs3": [[20., -20.], [60., -20.], [60., -60.], [20., -60.]],
		"obs4": [[-60., -20.], [-20., -20.], [-20., -60.], [-60., -60.]],
		# "obs5": [[-10., 10.], [10., 10.], [10., -10.], [-10., -10.]],
	}
	exp_obstacles = {
		"obs1": [[50., 90.], [40., 60.], [10., 60.], [20., 90.]],
		"obs2": [[10., 10.], [30., 10.], [30., -30.], [10., -30.]],
		"obs3": [[-70., 20.], [-40., 10.], [-70., -30.]],
		"obs4": [[-20., 90.], [-20., 40.], [-50., 70.]],
	}
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
