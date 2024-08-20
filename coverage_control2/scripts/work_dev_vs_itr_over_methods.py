import os
import sys
import time
import json
import signal
import threading
import traceback
import numpy as np

from pylab import plot, show, savefig, xlim, figure, hold, ylim, legend, boxplot, setp, axes

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

title_dict = {'geometric': 'Euclidean Lloyd', 
			  'geodesic_approximate': 'Euclidean Straight Skeleton Lloyd', 
			  'continuous_dijkstra': 'Geodesic Metric Grid Lloyd'}

fig_title = {"six_with_hole": "Six", 
			 "weird_e": "Corridors", 
			 "spiral": "Spiral",
			 "square_cvx": "Square"}

# method_labels = {'geometric': 'EL', 
# 				 'geodesic_approximate': 'SL', 
# 				 'continuous_dijkstra': 'GL'}

method_labels = {'geometric': 'Lloyd', 
				 'geodesic_approximate': 'SS', 
				 'continuous_dijkstra': 'MG'}

def get_total_path_traversed(pos_history):
	total = 0.
	for i in range(len(pos_history) - 1):
		total += np.linalg.norm(np.array(pos_history[i + 1]) - np.array(pos_history[i]))

	return total

if __name__ == "__main__":
	environment_name = sys.argv[1]

	exp_data_path = os.environ["HOME"] + "/thesis_ws/results/{}".format(environment_name)
	# exp_data_path = os.environ["HOME"] + "/Desktop/BackToTheCoverage/THESIS/results/{}".format(environment_name)
	results_table = dict()
	whole_results_list = dict()
	path_lengths = dict()
	all_path_lengths = dict()

	for exp_method in title_dict.keys():
		results_table[exp_method] = dict()
		path_lengths[exp_method] = dict()

		is_discrete = exp_method == "continuous_dijkstra"
		history_length = 35 if exp_method == "continuous_dijkstra" else 305

		for count in [4, 8]:
			results_table[exp_method][count] = dict()
			results_table[exp_method][count]["regular"] = []
			results_table[exp_method][count]["scaled"] = []
			path_lengths[exp_method][count] = dict()

			workload_scale = 0

			res_path = "{}/{}/{}".format(exp_data_path, exp_method, count)
			res_files = os.listdir(res_path)
			for res_file in res_files:
				if not res_file.endswith(".json"):
					continue

				with open("{}/{}".format(res_path, res_file), "r+") as E:
					exp_run = json.load(E)
					work_dev_hist = []
					scaled_work_dev_hist = []

					if workload_scale == 0:
						workload_scale = exp_run["workload_scale"]

					final_vals = [exp_run[str(j)]["workloads"][history_length - 1] 
									for j in xrange(1, count + 1)]

					path_lengths[exp_method][count] = dict([(j, get_total_path_traversed(exp_run[str(j)]["position"])) 
						for j in xrange(1, count + 1)])

					results_table[exp_method][count]["regular"].append(np.std(final_vals))

					if is_discrete:
						scaled_vals = [exp_run[str(j)]["workloads"][history_length - 1] * workload_scale ** 2
									for j in xrange(1, count + 1)]

						results_table[exp_method][count]["scaled"].append(np.std(scaled_vals))

			result_key = "{}_{}".format(exp_method, count)

			if not is_discrete:
				whole_results_list[result_key] = results_table[exp_method][count]["regular"]

			else:
				whole_results_list[result_key] = results_table[exp_method][count]["scaled"]

			all_path_lengths[result_key] = 

			mid = len(whole_results_list[result_key]) // 2
			whole_results_list[result_key].sort()
			lowest = min(whole_results_list[result_key])
			highest = max(whole_results_list[result_key])
			median = (whole_results_list[result_key][mid] + whole_results_list[result_key][~mid]) / 2

			print("{} [Workload Dist.] - Best: {:.3f} - Worst: {:.3f} - Median: {:.3f} - Mean: {:.3f} - Std.Dev.: {:.3f}".format(
				result_key, lowest, highest, median, np.mean(whole_results_list[result_key]), np.std(whole_results_list[result_key])))
			print("{} [Path Traversed] - Best: {:.3f} - Worst: {:.3f} - Median: {:.3f} - Mean: {:.3f} - Std.Dev.: {:.3f}".format(
				result_key))

			# if is_discrete:
			# 	scaled_result_key = "{}_{}_scaled".format(exp_method, count)
			# 	whole_results_list[scaled_result_key] = results_table[exp_method][count]["scaled"]

	fig = figure()
	ax = axes()

	# assert len(whole_results_list) == 8, "Exp. count is {}! Missing experiments?".format(len(whole_results_list))

	colors = ['pink', 'lightblue', 'lightgreen', 'orange']

	i = 1
	k = 0
	ticks = []
	tick_labels = []
	for exp_method in title_dict.keys():
		tick_labels.append(method_labels[exp_method])
		frames = []

		for count in [4, 8]:
			res_key = "{}_{}".format(exp_method, count)
			frames.append(whole_results_list[res_key])

		bps = boxplot(frames, positions=[i - 0.25, i + 0.25], 
				widths=0.4, patch_artist=True, boxprops=dict(facecolor=colors[k]))

		ticks.append(i)
		i += 3
		k += 1

		# if exp_method == "continuous_dijkstra":
		# 	tick_labels.append(method_labels[exp_method] + "/M")
		# 	scaled_frames = []

		# 	for count in [4, 8]:
		# 		scaled_res_key = "{}_{}_scaled".format(exp_method, count)
		# 		scaled_frames.append(whole_results_list[scaled_res_key])

		# 	bps = boxplot(scaled_frames, positions=[i - 0.25, i + 0.25], 
		# 			widths=0.4, patch_artist=True, boxprops=dict(facecolor=colors[k]))

		# 	ticks.append(i)
		# 	i += 2
		# 	k += 1

	ax.set_title(fig_title[environment_name])
	ax.set_xlabel("Methods")
	ax.set_ylabel("Variance")

	ax.set_xticks(ticks)
	ax.set_xticklabels(tick_labels)
	ax.set_xlim(0., 8.)

	figure_path = exp_data_path + "/work_std_dev_vs_agent_count_overall.png"
	savefig(figure_path, bbox_inches="tight")

	show()