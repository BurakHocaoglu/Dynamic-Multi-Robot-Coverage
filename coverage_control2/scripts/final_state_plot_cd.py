# Run this with conda environment created with Python 3 and scikit-geometry

import sys
import time
import copy
import json
import signal
import threading
import traceback
import numpy as np
# import skgeom as sg
import shapely.geometry as shgeom

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from functools import partial
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.5,0.5,0.5), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.99,0.99,0)]

valid_actions_4 = [(-1, 0, 1.),
				   (0, -1, 1.),
				   (0, 1, 1.),
				   (1, 0, 1.)]

valid_actions_8 = [(-1, -1, np.sqrt(2)), 
				   (-1, 0, 1.),
				   (-1, 1, np.sqrt(2)),
				   (0, -1, 1.),
				   (0, 1, 1.),
				   (1, -1, np.sqrt(2)),
				   (1, 0, 1.),
				   (1, 1, np.sqrt(2))]

history = None
limits = None
region_patch = None
region_patch2 = None
history_index = -1
history_thread = None
obstacle_patches = dict()
obstacle_patches2 = dict()

contracted_poly = None

class BFSAgent:

	def __init__(self, aid, pos, gpos, step_size):
		self.id = aid
		self.p = pos
		self.gp = gpos
		self.step_size = step_size

		self.visited = set()
		self.frontier = set()
		self.parents = dict()
		self.borders = dict()
		self.neighbour_info = dict()
		# self.edges = dict()
		self.edges = set()
		self.dvalue = 0

		self.frontier.add(tuple(self.gp))
		self.visited.add(tuple(self.gp))
		self.parents[tuple(self.gp)] = None
		self.normals = dict()
		self.orphan_border_vertices = []

	def frontier_expand(self):
		if globals()["contracted_poly"] is None:
			return None

		if len(self.frontier) == 0:
			return None

		next_wave = set()

		while len(self.frontier):
			f_pos = self.frontier.pop()

			# if self.edges.get(f_pos) is None:
			# 	self.edges[f_pos] = set()

			relative_expansion = []
			env_limited = False
			for act in globals()["valid_actions_4"]:
				move, cost = np.array(act[:2]) * self.step_size, act[2] * self.step_size

				next_pos = np.array(f_pos) + move
				if not globals()["contracted_poly"].contains(shgeom.Point(next_pos)):
					env_limited = True
					continue

				act_edge = (f_pos[0], f_pos[1], next_pos[0], next_pos[1])
				act_edge_rev = (next_pos[0], next_pos[1], f_pos[0], f_pos[1])

				if (act_edge not in self.edges) and (act_edge_rev not in self.edges):
					self.edges.add(act_edge)

				if tuple(next_pos) in self.visited:
					continue

				self.visited.add(tuple(next_pos))
				relative_expansion.append(tuple(next_pos))

				next_wave.add(tuple(next_pos))
				self.parents[tuple(next_pos)] = f_pos

			if not env_limited and len(relative_expansion) == 0:
				self.orphan_border_vertices.append(f_pos)

		self.frontier |= next_wave
		self.dvalue += 1
		return self.frontier

def check_emptiness(agents):
	total_emptiness = True

	for _, agent in agents.items():
		if len(agent.frontier) > 0:
			total_emptiness = False
			break

	return total_emptiness

def partitioned_view(agent_count, positions, resolution, outer_boundary, holes=dict()):
	poly = shgeom.Polygon(outer_boundary, list(holes.values()))
	globals()["contracted_poly"] = poly.buffer(- resolution / 2.)

	xmin, ymin, xmax, ymax = poly.bounds
	limits = poly.bounds
	xidx = xmin
	valid_metric_points = []

	gw = int(round((xmax - xmin) / resolution))
	gh = int(round((ymax - ymin) / resolution))

	r2g = lambda p: (int(round((p[1] - ymin) / resolution)), int(round((p[0] - xmin) / resolution)))
	g2r = lambda q: (resolution * q[1] + ymin, resolution * q[0] + xmin)
	coord_valid = lambda r, G: 0 <= r[0] < G.shape[0] and 1 <= r[1] < G.shape[1]

	while xidx < xmax:
		yidx = ymin

		while yidx < ymax:
			p = shgeom.Point(xidx, yidx)

			if globals()["contracted_poly"].contains(p):
				valid_metric_points.append((xidx, yidx))
				gidx = r2g((xidx, yidx))

			yidx += resolution
		xidx += resolution

	bfs_agents = dict()
	metric_partition = dict([(i, []) for i in range(1, agent_count + 1)])
	metric_assignment = dict()

	for i in range(1, agent_count + 1):
		rp_in_gp = np.array(valid_metric_points[0])
		min_gp_dist = np.linalg.norm(rp_in_gp - positions[i - 1])

		for j in range(1, len(valid_metric_points)):
			v = np.array(valid_metric_points[j])
			dist = np.linalg.norm(v - positions[i - 1])
			if dist < min_gp_dist:
				min_gp_dist = dist
				rp_in_gp = v

		bfs_agents[i] = BFSAgent(i, positions[i - 1], rp_in_gp, resolution)
		bfs_agents[i].borders = dict([(k, set([])) for k in range(1, agent_count + 1) if k != i])

	comp_start = time.time()
	while not check_emptiness(bfs_agents):
		deletion_updates = set()

		for k, agent in bfs_agents.items():
			expansions = agent.frontier_expand()

			if expansions is None:
				continue

			if len(expansions) == 0:
				pass

			for expansion in expansions:
				if metric_assignment.get(expansion) is None:
					metric_assignment[expansion] = k

				else:
					if metric_assignment[expansion] != k:
						deletion_updates.add((expansion, k))
						parent_of_deletion = agent.parents[expansion]
						normal = np.array(expansion) - np.array(parent_of_deletion)

						bfs_agents[k].borders[metric_assignment[expansion]].add(parent_of_deletion)
						bfs_agents[k].normals[parent_of_deletion] = normal

						bfs_agents[metric_assignment[expansion]].borders[k].add(expansion)
						bfs_agents[metric_assignment[expansion]].normals[expansion] = - normal

		for i in range(1, agent_count + 1):
			f_i = bfs_agents[i].frontier

			for j in range(i + 1, agent_count + 1):
				f_j = bfs_agents[j].frontier

				f_intersection = f_i & f_j

				# if len(f_intersection) > 0:
				# 	print("\nAgents {} & {}: {}\n".format(i, j, f_intersection))

				for f_vmp in f_intersection:
					deletion_updates.add((f_vmp, i))
					deletion_updates.add((f_vmp, j))

		for d_upd in deletion_updates:
			try:
				bfs_agents[d_upd[1]].frontier.remove(d_upd[0])
			except Exception as e:
				pass

	for k, agent in bfs_agents.items():
		for obv in agent.orphan_border_vertices:
			if metric_assignment[obv] != k:
				continue

			expansion_neighbours = []
			for act in valid_actions_4:
				move = np.array(act[:2]) * agent.step_size
				nb_expand = np.array(obv) + move

				if not globals()["contracted_poly"].contains(shgeom.Point(nb_expand)):
					continue

				assigned_bfs_id = metric_assignment.get(tuple(nb_expand))
				if assigned_bfs_id is None or assigned_bfs_id == k:
					continue

				expansion_neighbours.append(assigned_bfs_id)

			if len(expansion_neighbours) > 0:
				agent.borders[min(expansion_neighbours)].add(obv)

	for vmp, aid in metric_assignment.items():
		metric_partition[aid].append(vmp)

	return metric_partition

def compute_area_workload(poly, holes=[]):
	sh_poly = shgeom.Polygon(poly)
	outer_W = sh_poly.area

	for hole in holes:
		sh_hole = shgeom.Polygon(hole)
		outer_W -= sh_hole.area

	return outer_W

def get_total_path_traversed(pos_history):
	total = 0.
	for i in range(len(pos_history) - 1):
		total += np.linalg.norm(np.array(pos_history[i + 1]) - np.array(pos_history[i]))

	return total

if __name__ == "__main__":
	with open(sys.argv[1], "r") as H:
		history = json.load(H)

	if history is None:
		print("Could not load history content!")
		sys.exit(1)

	agent_count = history["agent_count"]
	if agent_count > len(__COLORS):
		additional_colors = [tuple(np.random.rand(3)) for j in range(agent_count - len(__COLORS) + 1)]
		__COLORS.extend(additional_colors)

	region_patch = plt.Polygon(history["coverage_region"], fill=False, color=(0., 0., 0.))
	region_patch2 = plt.Polygon(history["coverage_region"], fill=False, color=(0., 0., 0.))

	xcoords, ycoords = zip(*(history["coverage_region"]))
	xmin, xmax = min(xcoords), max(xcoords)
	ymin, ymax = min(ycoords), max(ycoords)
	limits = (xmin, ymin, xmax, ymax)

	all_holes = dict()
	if history.get("coverage_name") is not None:
		if (len(history["coverage_name"]) != 0) or history["coverage_name"] != "square_cvx":
			all_holes = history["coverage_obstacles"]

	for obs_name, obs in all_holes.items():
		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)
		obstacle_patches2[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)

	# -----------------------------------------------------------------------------------------------

	common_history_start = 5
	common_history_end = min([len(history[str(aid + 1)]["position"]) for aid in range(history["agent_count"])])
	print(f"Common history ends at step {common_history_end}.")

	# -----------------------------------------------------------------------------------------------

	figure2 = plt.figure()
	ax2 = figure2.add_subplot(1, 1, 1)

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
		area_loads = []

		resolution = 2. * history["workload_scale"]

		for aid in range(history["agent_count"]):
			pos = history[str(aid + 1)]["position"][-1]
			robot_color = __COLORS[aid + 1]
			ax2.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))

			x_hist, y_hist = zip(*(history[str(aid + 1)]["position"]))
			ax2.plot(x_hist, y_hist, color=robot_color, linewidth=2., alpha=0.5)

		# workloads = np.array([history[str(aid + 1)]["workloads"][common_history_start:common_history_end] for aid in range(history["agent_count"])])
		for aid in range(history["agent_count"]):
			workloads_aid = [history[str(aid + 1)]["workloads"][0]]

			for i in range(1, common_history_end):
				workload_aid_i = history[str(aid + 1)]["workloads"][i]
				workloads_aid.append(workload_aid_i if workload_aid_i > 4. else history[str(aid + 1)]["workloads"][i - 1])

			workloads.append(workloads_aid)

		workloads = np.array(workloads)
		# workloads = np.array([history[str(aid + 1)]["workloads"][:common_history_end] for aid in range(history["agent_count"])])
		area_loads = workloads * resolution ** 2

		metric_partition = partitioned_view(
								history["agent_count"], 
								[np.array(history[str(aid + 1)]["position"][-1]) for aid in range(history["agent_count"])], 
								resolution, 
								np.array(history["coverage_region"]), 
								all_holes)

		final_results = dict()
		for aid, partition in metric_partition.items():
			if len(partition) == 0:
				print(f"Empty partition for {aid}!")
				continue

			mpx, mpy = zip(*partition)
			ax2.scatter(mpx, mpy, s=8, color=__COLORS[aid], alpha=0.5)

			# Vertex count, equivalent area stat, max geodesic distance, total traversed distance
			final_results[aid] = (
				len(partition), 
				len(partition) * resolution ** 2, 
				max([np.linalg.norm(np.array(pos) - np.array(p)) for p in partition]), 
				# get_total_path_traversed(history[str(aid)]['position'][common_history_start:common_history_end]))
				get_total_path_traversed(history[str(aid)]['position'][:common_history_end]))

			print("---")
			print(f"Agent {aid} has:")
			print(f"\t* #Vertices : {final_results[aid][0]}")
			print(f"\t* Area      : {final_results[aid][1]}")
			print(f"\t* Max. Dist.: {final_results[aid][2]}")
			print(f"\t* #Distance : {final_results[aid][3]}")

		print("***")
		print(f"Total vertices: {sum([final_results[aid][0] for aid in metric_partition])}")
		print(f"Total area    : {sum([final_results[aid][1] for aid in metric_partition])}")

		if len(workloads) > 0:
			print(f"Workload shape    : {workloads.shape}")

			# std = np.std(workloads[-1])

			print("Workload Std. Dev. : {}".format(np.std(workloads[-1])))
			# print("Workload Std. - Var.: {} - {}".format(std, std ** 2))
			print("Area load Std. Dev.: {}".format(np.std(area_loads[-1])))

			if history.get("T_start") is not None:
				print(f"Experiment duration: {history['T_end'] - history['T_start']}")

	plt.savefig('final_view.png', bbox_inches='tight')

	plt.show()
