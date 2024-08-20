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

environment_scales = {
	"six_with_hole": 2.,
	"weird_e": 2.,
	"spiral": 1.
}

def compute_workload(env, space, resolution=1.):
	region = sg.Polygon(globals()["environment_boundary"][env])
	holes = globals()["environment_obstacles"][env]

	if space == 'c':
		negative_workload = 0.

		for _, hole in holes.items():
			negative_workload += sg.Polygon(hole).area

		return region.area - negative_workload

	elif space == 'd':
		poly = sg.Polygon(region, list(holes.values()))
		contracted_poly = poly.buffer(- resolution / 2.)

		xmin, ymin, xmax, ymax = poly.bounds
		xidx = xmin + resolution / 2.
		valid_metric_points = []

		while xidx < xmax:
			yidx = ymin + resolution / 2.

			while yidx < ymax:
				p = sg.Point(xidx, yidx)

				if poly.contains(p):
					valid_metric_points.append((xidx, yidx))

				yidx += resolution
			xidx += resolution

		return len(valid_metric_points)

if __name__ == "__main__":
	environment_name = sys.argv[1]
	# method_name = sys.argv[2]

	# exp_data_path = os.environ["HOME"] + "/Desktop/BackToTheCoverage/THESIS/results/{}/{}".format(
	# 														environment_name, method_name)
	exp_data_path = os.environ["HOME"] + "/Desktop/BackToTheCoverage/THESIS/results/{}".format(
															environment_name)

	results_table = dict()
	best_works = dict()
	worst_works = dict()

	for exp_method in title_dict.keys():
		results_table[exp_method] = dict()
		best_works[exp_method] = dict()
		worst_works[exp_method] = dict()

		is_discrete = exp_method == "continuous_dijkstra"
		history_length = 305

		if exp_method == "continuous_dijkstra":
			history_length = 35

			if environment_name == "spiral":
				history_length = 70

		for count in [4, 8]:
			results_table[exp_method][count] = []
			best_works[exp_method][count] = 0
			worst_works[exp_method][count] = 0

			workload_scale = 0

			res_path = "{}/{}/{}".format(exp_data_path, exp_method, count)
			res_files = os.listdir(res_path)
			for res_file in res_files:
				if not res_file.endswith(".json"):
					continue

				with open("{}/{}".format(res_path, res_file), "r+") as E:
					exp_run = json.load(E)
					work_dev_hist = []

					if workload_scale == 0:
						workload_scale = exp_run["workload_scale"]

					works = []
					for i in xrange(history_length):
						if is_discrete:
							works_i = [exp_run[str(j)]["workloads"][i] * workload_scale ** 2 for j in xrange(1, count + 1)]

						else:
							works_i = [exp_run[str(j)]["workloads"][i] for j in xrange(1, count + 1)]

						works.append(np.std(works_i))

						current_best = best_works[exp_method][count]
						if np.std(works_i) < np.std(works[current_best]):
							best_works[exp_method][count] = i

						# current_worst = worst_works[exp_method][count]
						# if np.std(works_i) > np.std(works[current_worst]):
						# 	worst_works[exp_method][count] = i

					results_table[exp_method][count].append(works)

	# ------------------------------------------------------------------------------------------
	colors = { 'geometric': 'red', 
			   'geodesic_approximate': 'blue', 
			   'continuous_dijkstra': 'green'}

	styles = { 'geometric': '', 
			   'geodesic_approximate': 'X', 
			   'continuous_dijkstra': 'D'}

	history_multiplier = 0.2
	history_length = 35
	if environment_name == "spiral":
		history_length = 70
		history_multiplier = 2.

	hist_max = history_length * history_multiplier

	pink_patch = patches.Patch(color='red', label='Lloyd')
	blue_patch = patches.Patch(color='blue', label='SS')
	green_patch = patches.Patch(color='green', label='MG')

	fig_a4 = plt.figure(num=1)
	ax1 = fig_a4.add_subplot(1, 1, 1)
	ax1.set_title('{} - 4 Agents'.format(fig_title[environment_name]))
	# ax1.set_xlabel('Iterations')
	ax1.set_xlabel('Running Time (s)')
	ax1.set_ylabel('Workload Std. Dev.')
	# ax1.set_xlim(0, 305)
	# ax1.set_xlim(0, history_length)
	ax1.set_xlim(0, hist_max)

	ax1.legend(handles=[pink_patch, blue_patch, green_patch])

	x_axis = np.arange(1, history_length + 1) * history_multiplier

	for exp_method in title_dict.keys():
		# print("{} - {} runs - {} per".format(exp_method, len(results_table[exp_method][4]), 
		# 	len(results_table[exp_method][4][0]) ))
		# for j in xrange(len(results_table[exp_method][4])):
		# 	ax1.plot(x_axis, results_table[exp_method][4][j][:history_length], color=colors[exp_method])
		best_idx = best_works[exp_method][4]
		ax1.plot(x_axis, results_table[exp_method][4][best_idx][:history_length], "-", color=colors[exp_method])
		# worst_idx = worst_works[exp_method][4]
		# ax1.plot(x_axis, results_table[exp_method][4][worst_idx][:history_length], ".", color=colors[exp_method])

	plt.savefig(exp_data_path + "/workload_dev_vs_itr_compact_A4.png", bbox_inches="tight")

	# ------------------------------------------------------------------------------------------

	fig_a8 = plt.figure(num=2)
	ax2 = fig_a8.add_subplot(1, 1, 1)
	ax2.set_title('{} - 8 Agents'.format(fig_title[environment_name]))
	# ax2.set_xlabel('Iterations')
	ax2.set_xlabel('Running Time (s)')
	ax2.set_ylabel('Workload Std. Dev.')
	# ax2.set_xlim(0, 305)
	# ax2.set_xlim(0, history_length)
	ax2.set_xlim(0, hist_max)

	ax2.legend(handles=[pink_patch, blue_patch, green_patch])

	for exp_method in title_dict.keys():
		# print("{} - {} runs - {} per".format(exp_method, len(results_table[exp_method][8]), 
		# 	len(results_table[exp_method][8][0]) ))
		# for j in xrange(len(results_table[exp_method][8])):
		# 	ax2.plot(x_axis, results_table[exp_method][8][j][:history_length], color=colors[exp_method])
		best_idx = best_works[exp_method][8]
		ax2.plot(x_axis, results_table[exp_method][8][best_idx][:history_length], color=colors[exp_method])

	plt.savefig(exp_data_path + "/workload_dev_vs_itr_compact_A8.png", bbox_inches="tight")

	# ------------------------------------------------------------------------------------------

	# fig_a8 = plt.figure(num=2)
	# ax2 = fig_a8.add_subplot(1, 1, 1)
	# ax2.set_title('{} - 8 Agents'.format(fig_title[environment_name]))
	# ax2.set_xlabel('Iterations')
	# ax2.set_ylabel('Workload Std. Dev.')
	# ax2.set_xlim(0, 305)

	# ax2.legend(handles=[pink_patch, blue_patch, green_patch])

	# for exp_method in title_dict.keys():
	# 	# print("{} - {} runs - {} per".format(exp_method, len(results_table[exp_method][8]), 
	# 	# 	len(results_table[exp_method][8][0]) ))
	# 	for j in xrange(len(results_table[exp_method][8])):
	# 		x_axis = np.arange(1, len(results_table[exp_method][8][j]) + 1)
	# 		ax2.plot(x_axis, results_table[exp_method][8][j], color=colors[exp_method])

	# plt.savefig(exp_data_path + "/workload_dev_vs_itr_compact_A8.png", bbox_inches="tight")

	plt.show()