import os
import sys
import time
import rospy
import signal
import traceback
import threading

import numpy as np
import skgeom as cgal
import shapely.geometry as sg

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from functools import partial
from collections import deque

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

# Data variables
global_skeleton_graph = dict()

coverage_boundary = None
coverage_obstacles = dict()
coverage_agents = dict()
grid_cell_size = 0.5
coord_limits = None

coverage_environment = None
agent_states = dict()
agents = dict()
stop = False
motion_history = dict()

# Visualization variables
region_patch = None
obstacle_patches = dict()

def FloydWarshall(matrix):
	pass

class Agent:

	def __init__(self, aid, p0, gpos, states=dict()):
		self.id = aid
		self.p = p0
		self.gp = gpos
		self.state_table = states

		self.visited = set()
		self.frontier = set()
		self.parents = dict()
		self.borders = dict()

	def advertise(self):
		self.states[self.id] = self.p

	def expand(self, graph):
		if len(self.frontier) == 0:
			return None

		next_wave = set()

		while len(self.frontier):
			f_pos = self.frontier.pop()

			relative_expansion = []

			pass

			pass

	def step(self):
		pass

def get_random_positions(env, lims, count, separation=5.):
	valid_samples, i = [], 0
	low_limit = [lims[0] + 1., lims[1] + 1.]
	high_limit = [lims[2] - 1., lims[3] - 1.]

	while i < count:
		p = np.random.uniform(low_limit, high_limit, (2,)).round(3)
		valid = True

		if not env.contains(sg.Point(p)):
			continue

		for sample in valid_samples:
			if np.linalg.norm(p - sample) <= separation:
				valid = False
				break

		if valid:
			valid_samples.append(p)
			i += 1

	return valid_samples

def real_2_grid(pos, lims, resolution):
	return (round((pos[0] - lims[0]) / resolution), 
			round((pos[1] - lims[1]) / resolution))

def grid_2_real(pos, lims, resolution):
	return (pos[0] * resolution + lims[0], pos[1] * resolution + lims[1])

def compute_straight_skeleton_graph(lims, boundary, holes=[], resolution=0.5):
	graph, nodes, edges = dict(), dict(), dict()
	environment = cgal.PolygonWithHoles(cgal.Polygon(boundary), 
										[cgal.Polygon(hole) for hole in holes])

	arr = cgal.arrangement.Arrangement()
	for i in range(len(boundary)):
		j = (i + 1) % len(boundary)

		arr.insert(cgal.Segment(cgal.Point2(boundary[i][0], boundary[i][1]), 
								cgal.Point2(boundary[j][0], boundary[j][1])))

	for _, hole in holes.items():
		for i in range(len(hole)):
			j = (i + 1) % len(hole)

			arr.insert(cgal.Segment(cgal.Point2(hole[i][0], hole[i][1]), 
									cgal.Point2(hole[j][0], hole[j][1])))

	skel = cgal.skeleton.create_interior_straight_skeleton(environment)
	node_id = 0

	for h in skeleton.halfedges:
		if h.is_bisector:
			p1 = h.vertex.point
			p2 = h.opposite.vertex.point
			gp1 = real_2_grid(p1, lims, resolution)
			gp2 = real_2_grid(p2, lims, resolution)

			# ep1 = (p1.x(), p1.y())
			# ep2 = (p2.x(), p2.y())

			d = np.array(p1) - np.array(p2)
			norm = np.linalg.norm(d)
			d /= norm

			steps = norm / resolution

			for k in range(steps):
				pass

			if nodes.get(gp1) is None:
				nodes[gp1] = (node_id, p1)
				node_id += 1

			if nodes.get(gp2) is None:
				nodes[gp2] = (node_id, p2)
				node_id += 1

			if edges.get(ep1) is None:
				edges[ep1] = set()

			if edges.get(ep2) is None:
				edges[ep2] = set()

			edges[ep1].add((ep2))
			edges[ep2].add(ep1)

	# adjacency = np.full((node_id, node_id), np.inf)

	# for node, n1 in nodes.items():
	# 	adjacency[n1, n1] = 0.

	# 	for nb in edges[node]:
	# 		n2 = nodes[nb]
	# 		adjacency[n1, n2] = np.linalg.norm(np.array(node) - np.array(nb))

	graph["nodes"] = nodes
	graph["edges"] = edges
	# graph["adjacency"] = adjacency
	graph["arrangement"] = arr

	return graph

def animate_partitioning(i, ax, lims, S):
	ax.clear()
	ax.set_aspect("equal")
	ax.set_axis_off()
	ax.set_xlim(lims[0] - 5., lims[2] + 5.)
	ax.set_ylim(lims[1] - 5., lims[3] + 5.)

	ax.add_patch(globals()["region_patch"])

	for _, obs_patch in globals()["obstacle_patches"].items():
		ax.add_patch(obs_patch)

	for aid, state in S.items():
		pass

def partition_experiment(agents, global_skeleton, stop_flag):
	rospy.loginfo("Partitioning begins.")

	while not stop_flag:
		pass

	rospy.loginfo("Partitioning ends.")

def customSigIntHandler(signum, frame):
	rospy.logwarn("Stopping...")
	globals()["stop"] = False

if __name__ == "__main__":
	agent_count = int(sys.argv[1])
	assert agent_count > 1, "Not enough to experiment"

	rospy.init_node("skeletal_partitioning_node", anonymous=False, disable_signals=True)

	signal.signal(signal.SIGINT, customSigIntHandler)

	coverage_boundary = rospy.get_param("/coverage_boundary", [])
	xcoords, ycoords = zip(*coverage_boundary)
	# limits = (min(xcoords), min(ycoords), max(xcoords), max(ycoords))

	region_patch = plt.Polygon(list(coverage_boundary), fill=False, color=(0., 0., 0.))

	coverage_obstacles = rospy.get_param("/coverage_obstacles", dict())
	for obs_name, obs in coverage_obstacles.items():
		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)

	coverage_environment = sg.Polygon(coverage_boundary, coverage_obstacles.values())
	contracted_environment = coverage_environment.buffer(- grid_cell_size / 2.)
	limits = contracted_environment.bounds

	rospy.loginfo("Environment bounds (x-, y-, x+, y+): {}".format(limits))

	global_skeleton_graph = compute_straight_skeleton_graph(limits, coverage_boundary, 
								holes=coverage_obstacles, resolution=grid_cell_size)

	rospy.loginfo("Constructed global skeleton graph.")

	rand_positions = get_random_positions(contracted_environment, limits, 
										  agent_count, separation=5.)

	for i in range(1, agent_count + 1):
		p_i_real = rand_positions[i - 1]
		p_i_grid = real_2_grid(p_i_real, limits, grid_cell_size)

		agent[i] = Agent(i, p_i_real, p_i_grid, agent_states)

	partition_thread = threading.Thread(name="partitioner", target=partition_experiment, 
										args=(agents, global_skeleton_graph, stop))

	partition_thread.start()

	figure = plt.figure()
	exp_ax = figure.add_subplot(1, 1, 1)
	ani_func = animation.FuncAnimation(figure, partial(animate_experiment, 
														ax=exp_ax, 
														lims=limits, 
														S=all_states), interval=200)

	plt.show(block=True)

	rospy.spin()

	stop = True
	rospy.loginfo("Done!")
	partition_thread.join()