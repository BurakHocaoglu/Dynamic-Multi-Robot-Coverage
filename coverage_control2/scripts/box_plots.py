import os
import sys
import time
import json
# import signal
# import threading
import traceback
import numpy as np
import pandas as pd
import scipy.stats as st
# import shapely.geometry as sg

import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.patches as patches

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

# __METHODS = ['geometric', 'geodesic_approximate', 'continuous_dijkstra']
__METHODS = ['geometric', 'continuous_dijkstra']

# __METHOD_COLORS = ["blue", "red", "green"]
__METHOD_COLORS = ["blue", "green"]
# __METHOD_KEYS = ["Lloyd", "SS", "MG"]
__METHOD_KEYS = ["Lloyd", "MG"]

method_legend = {
	'geometric': 'Lloyd', 
	# 'geodesic_approximate': 'SS', 
	'continuous_dijkstra': 'MG'
}

if __name__ == "__main__":
	environment_name = sys.argv[1]
	exp_data_path = os.environ["HOME"] + "/thesis_ws/results/{}".format(environment_name)

	# frames = dict()
	workload_data = dict()
	exec_time_data = dict()
	assignment_data = dict()

	for method in __METHODS:
		if workload_data.get(method) is None:
			workload_data[method] = dict()
			exec_time_data[method] = dict()
			assignment_data[method] = dict()

		for count in [8, 16, 32]:
			files = os.listdir("{}/{}/{}".format(exp_data_path, method, count))

			for file in files:
				if not file.endswith(".json"):
					continue

				# n_steps = 0
				with open("{}/{}/{}/{}".format(exp_data_path, method, count, file), 'r') as D:
					exp_run = json.load(D)
					n_steps = exp_run["global_iteration_threshold"]
					stats = []

					for i in range(n_steps):
						works_i = [exp_run["agent_{}".format(aid)]["workload"][i] for aid in range(count)]
						works_mean = np.mean(works_i)
						works_i /= np.mean(works_mean)
						stats.append(np.std(works_i))

					if workload_data[method].get(count) is None:
						workload_data[method][count] = []
						exec_time_data[method][count] = []
						assignment_data[method][count] = []

					if len(stats) > 0:
						# workload_data[method][count].append(stats)
						workload_data[method][count].append(stats[-1])
						exec_time_data[method][count].append(exp_run["TIME"])

						assignment = exp_run.get("percentage")
						if assignment is not None:
							assignment_data[method][count].append(assignment)

					# if method == 'geodesic_approximate' and count == 32:
					# 	# workload_data[method][count].append()
					# 	pass

					# else:
					# 	workload_data[method][count].append(np.std(list(exp_run["overall"].values())))

	# -------------------------------------------------------------------------------
	# workload_data['geodesic_approximate'][32] = [10 for i in range(10)]

	title_name = environment_name
	# if title_name == "ROMER_v2":
	# 	title_name = "ROMER"

	w_figure = plt.figure(num=1)
	w_ax = w_figure.add_subplot(1, 1, 1)
	# w_ax.set_aspect("equal")
	w_ax.set_xlabel("# of Agents")
	w_ax.set_ylabel("Avg. Norm. Workload Variance")
	w_ax.set_title(title_name)

	Lloyd = pd.DataFrame({
		8: workload_data["geometric"][8],
		16: workload_data["geometric"][16],
		32: workload_data["geometric"][32]
	})

	Lloyd_A = pd.DataFrame({
		8: assignment_data["geometric"][8],
		16: assignment_data["geometric"][16],
		32: assignment_data["geometric"][32]
	})

	# SS = pd.DataFrame({
	# 	8: workload_data["geodesic_approximate"][8],
	# 	16: workload_data["geodesic_approximate"][16],
	# 	32: workload_data["geodesic_approximate"][32]
	# })

	# SS_A = pd.DataFrame({
	# 	8: assignment_data["geodesic_approximate"][8],
	# 	16: assignment_data["geodesic_approximate"][16],
	# 	32: assignment_data["geodesic_approximate"][32]
	# })

	MG = pd.DataFrame({
		8: workload_data["continuous_dijkstra"][8],
		16: workload_data["continuous_dijkstra"][16],
		32: workload_data["continuous_dijkstra"][32]
	})

	MG_A = pd.DataFrame({
		8: assignment_data["continuous_dijkstra"][8],
		16: assignment_data["continuous_dijkstra"][16],
		32: assignment_data["continuous_dijkstra"][32]
	})

	# w_frames = [Lloyd, SS, MG]
	w_frames = [Lloyd, MG]
	p_frames = [Lloyd_A, MG_A]
	w_legend = []
	p_legend = []
	x_pos_range = np.arange(len(w_frames)) / (len(w_frames) - 1)
	x_pos = (x_pos_range * 0.5) + 0.75

	for i, data in enumerate(w_frames):
		# bp = plt.boxplot(np.array(data), sym='', whis=[0, 100], 
		# 	widths=0.5 / len(w_frames), labels=[8, 16, 32], patch_artist=True,
		# 	positions=[x_pos[i] + j * 1 for j in range(len(data.T))])
		bp = w_ax.boxplot(np.array(data), sym='', whis=[0, 100], 
			widths=0.5 / len(w_frames), labels=[8, 16, 32], patch_artist=True,
			positions=[x_pos[i] + j * 1 for j in range(len(data.T))])

		for box in bp["boxes"]:
			box.set(facecolor=__METHOD_COLORS[i])

		w_legend.append(patches.Patch(facecolor=__METHOD_COLORS[i], label=__METHOD_KEYS[i]))

	# for i, data in enumerate(p_frames):
	# 	# bp = plt.boxplot(np.array(data), sym='', whis=[0, 100], 
	# 	# 	widths=0.5 / len(w_frames), labels=[8, 16, 32], patch_artist=True,
	# 	# 	positions=[x_pos[i] + j * 1 for j in range(len(data.T))])
	# 	bp = p_ax.boxplot(np.array(data), sym='', whis=[0, 100], 
	# 		widths=0.5 / len(p_frames), labels=[8, 16, 32], patch_artist=True,
	# 		positions=[x_pos[i] + j * 1 for j in range(len(data.T))])

	# 	for box in bp["boxes"]:
	# 		box.set(facecolor=__METHOD_COLORS[i])

	# 	w_legend.append(patches.Patch(facecolor=__METHOD_COLORS[i], label=__METHOD_KEYS[i]))

	plt.xticks(np.arange(len(list(w_frames[0]))) + 1)
	plt.gca().xaxis.set_minor_locator(ticker.FixedLocator(
		np.array(range(len(list(w_frames[0])) + 1)) + 0.5))

	plt.gca().tick_params(axis='x', which='minor', length=4)
	plt.gca().tick_params(axis='x', which='major', length=0)

	plt.legend(handles=w_legend, fontsize=8)

	plt.savefig("{}/avg_norm_workload_var_{}.png".format(exp_data_path, environment_name), 
		bbox_inches="tight")

	plt.close(w_figure)

	# -------------------------------------------------------------------------------

	p_figure = plt.figure(num=2)
	p_ax = p_figure.add_subplot(1, 1, 1)
	# p_ax.set_aspect("equal")
	p_ax.set_xlabel("# of Agents")
	p_ax.set_ylabel("Assignment")
	p_ax.set_title(title_name)

	Lloyd_A = pd.DataFrame({
		8: assignment_data["geometric"][8],
		16: assignment_data["geometric"][16],
		32: assignment_data["geometric"][32]
	})

	# SS_A = pd.DataFrame({
	# 	8: assignment_data["geodesic_approximate"][8],
	# 	16: assignment_data["geodesic_approximate"][16],
	# 	32: assignment_data["geodesic_approximate"][32]
	# })

	# MG_A = pd.DataFrame({
	# 	8: assignment_data["continuous_dijkstra"][8],
	# 	16: assignment_data["continuous_dijkstra"][16],
	# 	32: assignment_data["continuous_dijkstra"][32]
	# })

	p_frames = [Lloyd_A]
	p_legend = []
	x_pos_range = np.arange(len(w_frames)) / (len(w_frames) - 1)
	x_pos = (x_pos_range * 0.5) + 0.75

	for i, data in enumerate(p_frames):
		bp = p_ax.boxplot(np.array(data), sym='', whis=[0, 100], 
			widths=0.5 / len(p_frames), labels=[8, 16, 32], patch_artist=True,
			positions=[x_pos[i] + j * 1 for j in range(len(data.T))])

		for box in bp["boxes"]:
			box.set(facecolor=__METHOD_COLORS[i])

		p_legend.append(patches.Patch(facecolor=__METHOD_COLORS[i], label=__METHOD_KEYS[i]))

	plt.xticks(np.arange(len(list(p_frames[0]))) + 1)
	plt.gca().xaxis.set_minor_locator(ticker.FixedLocator(
		np.array(range(len(list(p_frames[0])) + 1)) + 0.5))

	plt.gca().tick_params(axis='x', which='minor', length=4)
	plt.gca().tick_params(axis='x', which='major', length=0)

	plt.legend(handles=p_legend, fontsize=8)

	plt.savefig("{}/assignment_percentage_{}.png".format(exp_data_path, environment_name), 
		bbox_inches="tight")

	plt.close(p_figure)

	# -------------------------------------------------------------------------------
