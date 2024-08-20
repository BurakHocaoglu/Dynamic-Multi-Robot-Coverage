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
	method_name = sys.argv[2]
	agent_count = int(sys.argv[3])

	exp_data_path = os.environ["HOME"] + "/thesis_ws/results/{}/{}/{}".format(environment_name, 
																			  method_name, 
																			  agent_count)
	# exp_data_path = os.environ["HOME"] + "/Desktop/BackToTheCoverage/THESIS/results/{}/{}/{}".format(
	# 														environment_name, method_name, agent_count)
	# exp_data_path = os.environ["HOME"] + "/Desktop/BackToTheCoverage/THESIS/results/{}/{}".format(
	# 														environment_name, method_name)

	is_discrete = method_name == "continuous_dijkstra"
	history_length = 35 if method_name == "continuous_dijkstra" else 305
	workload_scale = 0
	global_workload = 0

	if environment_name == "spiral" and method_name == "continuous_dijkstra":
		history_length = 70

	experimental_runs = dict()
	workload_deviation_history = dict()
	scaled_workload_deviation_history = dict()
	# coverage_ratio_history = dict()

	# for agent_count in [4, 8]:
	# 	pass

	data_files = os.listdir(exp_data_path)
	for data_file in data_files:
		if not data_file.endswith('.json'):
			continue

		try:
			with open("{}/{}".format(exp_data_path, data_file), "r+") as E:
				experimental_runs[data_file] = json.load(E)
				workload_deviation_history[data_file] = []
				scaled_workload_deviation_history[data_file] = []
				# coverage_ratio_history[data_file] = []

				if workload_scale == 0:
					workload_scale = experimental_runs[data_file]["workload_scale"]
					global_workload = compute_workload(environment_name, 
														'd' if is_discrete else 'c', 
														environment_scales[environment_name])

					print("---")
					print("Global workload: {}".format(global_workload))
					print("Workload scale : {}".format(workload_scale))
					print("---")

					# global_workload *= workload_scale

				for i in xrange(history_length):
					works_i = [experimental_runs[data_file][str(j)]["workloads"][j] 
									for j in xrange(1, agent_count + 1)]

					workload_deviation_history[data_file].append(np.std(works_i))
					# coverage_ratio_history[data_file].append(np.sum(works_i) / global_workload)

					if is_discrete:
						scaled_works_i = [experimental_runs[data_file][str(j)]["workloads"][i] * workload_scale ** 2
												for j in xrange(1, agent_count + 1)]

						scaled_workload_deviation_history[data_file].append(np.std(scaled_works_i))

		except Exception as e:
			print("File: {}".format(data_file))
			print(traceback.format_exc())

	fig = plt.figure(num=1)
	ax = fig.add_subplot(1, 1, 1)

	ax.set_title('{} - {} Agents'.format(fig_title[environment_name], agent_count))
	ax.set_xlabel('Iterations')
	ax.set_ylabel('Workload Std. Dev.')

	if is_discrete:
		red_patch = patches.Patch(color='red', label='Count based')
		blue_patch = patches.Patch(color='blue', label='Mass/Area based')
		ax.legend(handles=[red_patch, blue_patch])

	x_axis = np.arange(1, history_length + 1)

	for _, dev_hist in workload_deviation_history.items():
		ax.plot(x_axis, dev_hist, color=(0.99, 0., 0.))

	final_vals = [values[-1] for values in workload_deviation_history.values()]
	best_final, worst_final = min(final_vals), max(final_vals)

	if not is_discrete:
		print("Best final: {}".format(best_final))
		print("Worst final: {}".format(worst_final))
		print("Finals std. dev.: {}".format(np.std(final_vals)))
		ci = st.norm.interval(alpha=0.95, loc=np.mean(final_vals), scale=st.sem(final_vals))
		print("Finals CI (95): {} +/- {}".format(ci[0], ci[1]))

		figure_path = exp_data_path + "/workload_std_dev_vs_iteration_count.png"
		plt.savefig(figure_path, bbox_inches="tight")

	else:
		for _, scaled_dev_hist in scaled_workload_deviation_history.items():
			ax.plot(x_axis, scaled_dev_hist, color=(0., 0., 0.99))

		final_scaled_vals = [values[-1] for values in scaled_workload_deviation_history.values()]
		best_scaled_final, worst_scaled_final = min(final_scaled_vals), max(final_scaled_vals)

		print("Best final (count based): {}".format(best_final))
		print("Worst final (count based): {}".format(worst_final))
		print("Finals std. dev. (count based): {}".format(np.std(final_vals)))
		# ci = st.norm.interval(alpha=0.95, loc=np.mean(final_vals), scale=st.sem(final_vals))
		# print("Finals CI (95 - count based): {} +/- {}".format(ci[0], ci[1]))

		print("Best final (mass/area based): {}".format(best_scaled_final))
		print("Worst final (mass/area based): {}".format(worst_scaled_final))
		print("Finals std. dev. (mass/area based): {}".format(np.std(final_scaled_vals)))
		# s_ci = st.norm.interval(alpha=0.95,loc=np.mean(final_scaled_vals), scale=st.sem(final_scaled_vals))
		# print("Finals CI (95 - mass/area based): {} +/- {}".format(s_ci[0], s_ci[1]))

		figure_path = exp_data_path + "/workload_std_dev_vs_iteration_count.png"
		plt.savefig(figure_path, bbox_inches="tight")

	# print("---------")

	# rate_fig = plt.figure(num=2)
	# rate_ax = rate_fig.add_subplot(1, 1, 1)
	# rate_ax.set_title('{} - {} Agents [Percentage]'.format(fig_title[environment_name], agent_count))
	# rate_ax.set_xlabel("Iterations")
	# rate_ax.set_ylabel("Percentage")
	# rate_ax.set_ylim(0, 2)

	# for _, rate_hist in coverage_ratio_history.items():
	# 	rate_ax.plot(x_axis, rate_hist, color=(0.99, 0.5, 0.))

	# final_rate_vals = [values[-1] for values in coverage_ratio_history.values()]
	# best_final_rate, worst_final_rate = min(final_rate_vals), max(final_rate_vals)

	# print("Best final rate: {}".format(best_final_rate))
	# print("Worst final rate: {}".format(worst_final_rate))
	# print("Finals std. dev.: {}".format(np.std(final_rate_vals)))

	# rate_figure_path = exp_data_path + "/coverage_percentage_vs_iteration_count.png"
	# plt.savefig(rate_figure_path, bbox_inches="tight")

	plt.show()