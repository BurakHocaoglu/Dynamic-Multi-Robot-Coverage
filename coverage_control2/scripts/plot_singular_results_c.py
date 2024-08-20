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

title_dict = {
	'geometric': 'Baseline', 
	'geodesic_approximate': 'Method 1', 
	'continuous_dijkstra': 'Method 2'
}

fig_title = {
	"six_with_hole": "Six", 
	"weird_e": "Corridors", 
	"spiral": "Spiral",
	"square_cvx": "Square"
}

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

	"square_cvx": [[80., 80.], [-80., 80.], [-80., -80.], [80., -80.]],

	"ROMER": [[0., 10.], [60., 10.], [60., 80.], [65., 80.],
				[65., 10.], [115., 10.], [115., 80.], [120., 80.],
				[120., 10.], [170., 10.], [170., 80.], [175., 80.],
				[175., 10.], [225., 10.], [225., 145.], [155., 145.],
				[155., 115.], [150., 115.], [150., 180.], [155., 180.],
				[155., 150.], [225., 150.], [225., 240.], [155., 240.],
				[155., 210.], [150., 210.], [150., 240.], [127., 240.],
				[127., 210.], [122., 210.], [122., 240.], [100., 240.],
				[100., 210.], [95., 210.], [95., 240.], [67., 240.],
				[67., 210.], [62., 210.], [62., 240.], [0., 240.]]
}

environment_obstacles = {
	"six_with_hole": {
		"obs": [[-70., 70.], [10., 70.], [10., 20.], [-70., 20.]],
	},

	"weird_e": dict(),

	"spiral": dict(),

	"square_cvx": dict(),

	"ROMER": {
		"obs1": [[85., 180.], [125., 180.], [125., 155.], [105., 155.],
           [105., 135.], [125., 135.], [125., 115.], [85., 115.]],

    	"obs2": [[20., 180.], [60., 180.], [60., 115.], [20., 115.]]
	}
}

if __name__ == "__main__":
	environment_name = sys.argv[1]
	method_name = sys.argv[2]
	exp_data_path = os.environ["HOME"] + "/thesis_ws/results/{}/{}".format(environment_name, method_name)

	# for count in [8, 12, 16]:
	for count in [8, 16]:
		files = os.listdir("{}/{}".format(exp_data_path, count))
		workload_data = []
		max_geod_data = []

		fig_num = 1
		for file in files:
			if not file.endswith(".json"):
				continue

			# print("\tReading file {}...".format(file))
			workload_data_i = []
			# max_geod_data_i = []

			nSteps = 0
			with open("{}/{}/{}".format(exp_data_path, count, file), 'r') as D:
				exp_run = json.load(D)

				nSteps = exp_run["global_iteration_threshold"]
				x_axis = np.arange(1, nSteps + 1)

				exp_data = dict()
				for aid in range(count):
					exp_data[aid] = dict()
					exp_data[aid]["workload"] = exp_run["agent_{}".format(aid)]["workload"]
					exp_data[aid]["max_geod"] = exp_run["agent_{}".format(aid)]["max_geod"]
					# exp_data[aid]["actual_work"] = exp_run["agent_{}".format(aid)]["actual_work"]

				for j in range(nSteps):
					workload_j = [exp_data[aid]["workload"][j] for aid in range(count)]
					max_geod_j = [exp_data[aid]["max_geod"][j] for aid in range(count)]
					workload_data_i.append(np.std(workload_j))
					# max_geod_data_i.append(np.std(max_geod_j))
					# workload_data_i.append(np.mean(workload_j))
					# max_geod_data_i.append(np.mean(max_geod_j))

			workload_data.append(workload_data_i)
			# max_geod_data.append(max_geod_data_i)

		w_figure = plt.figure(num=fig_num)
		w_ax = w_figure.add_subplot(1, 1, 1)
		# w_ax.set_aspect("equal")
		w_ax.set_xlabel("Time")
		w_ax.set_ylabel("Workload Std. Dev.")
		w_ax.set_title("{} - {} Agents".format(environment_name, count))
		# w_ax.set_xlim(0., nSteps)

		k = 0
		for item in workload_data:
			w_ax.plot(np.arange(1, len(item) + 1), item, color=__COLORS[k], alpha=0.5)
			# w_ax.plot(item, color=(1., 0., 0.), alpha=0.5)
			k += 1

		plt.savefig("{}/{}/workload_std_dev.png".format(exp_data_path, count), bbox_inches="tight")


		g_figure = plt.figure(num=fig_num + 1)
		g_ax = g_figure.add_subplot(1, 1, 1)
		# g_ax.set_aspect("equal")
		g_ax.set_xlabel("Time")
		g_ax.set_ylabel("Max. Geod. Dist. Std. Dev.")
		g_ax.set_title("{} - {} Agents".format(environment_name, count))
		# g_ax.set_xlim(0., nSteps)

		k = 0
		for item in max_geod_data:
			g_ax.plot(np.arange(1, len(item) + 1), item, color=__COLORS[k], alpha=0.5)
			k += 1

		plt.savefig("{}/{}/max_geod_std_dev.png".format(exp_data_path, count), bbox_inches="tight")


		plt.close(w_figure)
		plt.close(g_figure)
		fig_num += 2

	# plt.show()
