import os
import sys
import time
import json
import signal
import threading
import traceback
import numpy as np
import scipy.stats as st
import shapely.geometry as sg

import matplotlib.pyplot as plt
import matplotlib.patches as patches

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

title_dict = {'geometric': 'Baseline', 
			  'geodesic_approximate': 'Method 1', 
			  'continuous_dijkstra': 'Method 2'}

fig_title = {"six_with_hole": "Six", 
			 "weird_e": "Corridors", 
			 "spiral": "Spiral",
			 "square_cvx": "Square"}

environment_boundary = {
	"six_with_hole": [[-80., 130.], [-80., 10.], [20., 10.], [20., 80.], 
					[-70., 80.], [-70., 120.], [20., 120.], [20., 130.]],

	"weird_e": [[-90., 120.], [-90., 10.], [-10., 10.], [-10., 20.], 
				[-80., 20.], [-80., 40.], [-10., 40.], [-10., 60.], 
				[-80., 50.], [-80., 70.], [-30., 60.], [-20., 80.], 
				[-80., 80.], [-80., 90.], [-10., 90.], [-10., 100.], 
				[-80., 100.], [-80., 110.], [-10., 110.], [-10., 120.]],

	"spiral": [[120., 140.], [50., 140.], [50., 90.], [115., 90.], 
				[115., 130.], [60., 130.], [60., 100.], [105., 100.], 
				[105., 120.], [70., 120.], [70., 115.], [100., 115.],
				[100., 105.], [65., 105.], [65., 125.], [110., 125.],
				[110., 95.], [55., 95.], [55., 135.], [120., 135.]],

	"square_cvx": [[80., 80.], [-80., 80.], [-80., -80.], [80., -80.]]
}

environment_obstacles = {
	"six_with_hole": {
		"obs": [[-70., 70.], [10., 70.], [10., 20.], [-70., 20.]],
	},
	"weird_e": dict(),
	"spiral": dict(),
	"square_cvx": dict()
}

if __name__ == "__main__":
	environment_name = sys.argv[1]
	method_name = sys.argv[2]
	agent_count = int(sys.argv[3])

	exp_data_path = os.environ["HOME"] + "/thesis_ws/results/{}/{}/{}".format(environment_name, 
																			  method_name, 
																			  agent_count)

	is_discrete = method_name == "continuous_dijkstra"
	history_length = 35 if method_name == "continuous_dijkstra" else 305
	workload_scale = 0

	if environment_name == "spiral" and method_name == "continuous_dijkstra":
		history_length = 70

	experimental_runs = dict()
	trajs = []

	data_files = os.listdir(exp_data_path)
	for data_file in data_files:
		if not data_file.endswith('.json'):
			continue

		try:
			with open("{}/{}".format(exp_data_path, data_file), "r+") as E:
				experimental_runs[data_file] = json.load(E)

				for j in xrange(1, agent_count + 1):
					traj = experimental_runs[data_file][str(j)]["position"]
					traj_x, traj_y = zip(*traj)
					trajs.append((traj_x, traj_y))

		except Exception as e:
			print("File: {}".format(data_file))
			print(traceback.format_exc())

	fig = plt.figure(num=1)
	ax = fig.add_subplot(1, 1, 1)

	ax.set_title('{} - {} Agents'.format(fig_title[environment_name], agent_count))
	ax.set_axis_off()
	ax.set_aspect("equal")

	ax.add_patch(plt.Polygon(environment_boundary[environment_name], fill=False, color=(0., 0., 0.)))

	if environment_name == "six_with_hole":
		for _, hole in environment_obstacles[environment_name].items():
			ax.add_patch(plt.Polygon(hole, fill=True, color=(0., 0., 0.)))

	for t in trajs:
		ax.plot(t[0], t[1], color=(1., 0., 0.))
		ax.plot([t[0][-1]], [t[1][-1]], 'o', color=(0., 0., 1.))

	figure_path = exp_data_path + "/movement_history.png"
	plt.savefig(figure_path, bbox_inches="tight")

	plt.show()