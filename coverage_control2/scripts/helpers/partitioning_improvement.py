import os
import sys
import time
import json
import signal
import threading
import traceback
import numpy as np
# import skgeom as sg
# import shapely.geometry as shgeom

import matplotlib.pyplot as plt
import matplotlib.patches as patches
# import matplotlib.animation as animation

# from datetime import datetime
# from functools import partial
# from matplotlib import cm
# from mpl_toolkits.mplot3d import Axes3D

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

title_dict = {'geometric': 'Euclidean Lloyd', 
			  'geodesic_approximate': 'Euclidean Straight Skeleton Lloyd', 
			  'continuous_dijkstra': 'Geodesic Metric Grid Lloyd'}

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

	experimental_runs = dict()
	workload_deviation_history = dict()
	scaled_workload_deviation_history = dict()

	data_files = os.listdir(exp_data_path)
	for data_file in data_files:
		with open("{}/{}".format(exp_data_path, data_file), "r+") as E:
			experimental_runs[data_file] = json.load(E)
			workload_deviation_history[data_file] = []
			scaled_workload_deviation_history[data_file] = []

			if workload_scale == 0:
				workload_scale = experimental_runs[data_file]["workload_scale"]

			for i in xrange(history_length):
				works_i = [experimental_runs[data_file][str(j)]["workloads"][i] 
								for j in xrange(1, agent_count + 1)]

				workload_deviation_history[data_file].append(np.std(works_i))

				if is_discrete:
					scaled_works_i = [experimental_runs[data_file][str(j)]["workloads"][i] * workload_scale ** 2
											for j in xrange(1, agent_count + 1)]

					scaled_workload_deviation_history[data_file].append(np.std(scaled_works_i))

	fig = plt.figure(num=1)
	ax = fig.add_subplot(1, 1, 1)

	ax.set_title(title_dict[method_name])
	ax.set_xlabel('Iterations')
	ax.set_ylabel('Workload Std. Dev')

	if is_discrete:
		red_patch = patches.Patch(color='red', label='Count based')
		blue_patch = patches.Patch(color='blue', label='Mass/Area based')
		ax.legend(handles=[red_patch, blue_patch])

	x_axis = np.arange(1, history_length + 1)

	for _, dev_hist in workload_deviation_history.items():
		ax.plot(x_axis, dev_hist, color=(0.99, 0., 0.))

	if not is_discrete:
		figure_path = exp_data_path + "/workload_std_dev_vs_iteration_count.png"
		plt.savefig(figure_path, bbox_inches="tight")

	else:
		for _, scaled_dev_hist in scaled_workload_deviation_history.items():
			ax.plot(x_axis, scaled_dev_hist, color=(0., 0., 0.99))

		figure_path = exp_data_path + "/workload_std_dev_vs_iteration_count.png"
		plt.savefig(figure_path, bbox_inches="tight")

	# figure_path = exp_data_path + "/workload_std_dev_vs_iteration_count.png"
	# plt.savefig(figure_path, bbox_inches="tight")

	plt.show()