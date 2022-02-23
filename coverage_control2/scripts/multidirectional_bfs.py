import time
import traceback
import numpy as np
import shapely.geometry as sg
import matplotlib.pyplot as plt

from collections import deque

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

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

class BFSAgent:

	def __init__(self, aid, pos, gpos, step_size):
		self.id = aid
		self.p = pos
		self.gp = gpos
		self.step_size = step_size

		self.visited = set()
		self.frontier = set()
		self.dvalue = 0

		self.frontier.add(tuple(self.gp))

	def frontier_expand(self):
		if len(self.frontier) == 0:
			return None

		next_wave = set()

		while len(self.frontier):
			f_pos = self.frontier.pop()
			self.visited.add(f_pos)

			for act in globals()["valid_actions_4"]:
				move, cost = np.array(act[:2]) * self.step_size, act[2] * self.step_size

				next_pos = np.array(f_pos) + move
				if not globals()["contracted_poly"].contains(sg.Point(next_pos)):
					continue

				if tuple(next_pos) in self.visited:
					continue

				next_wave.add(tuple(next_pos))

		self.frontier |= next_wave
		return self.frontier

def check_emptiness(agents):
	total_emptiness = True

	for _, agent in agents.items():
		if len(agent.frontier) > 0:
			total_emptiness = False
			break

	return total_emptiness

if __name__ == "__main__":
	resolution = 4.
	agent_count = 4

	outer_boundary = np.array([[-80., 60.], [-20., 60.], [-20., 40.], [20., 40.], 
								[20., 60.], [80., 60.], [80., 40.], [40., 40.], 
								[40., 20.], [80., 20.], [80., -40.], [40., -40.], 
								[40., -60.], [80., -60.], [80., -80.], [20., -80.], 
								[20., -60.], [-20., -60.], [-20., -80.], [-80., -80.], 
								[-80., -60.], [-40., -60.], [-40., -20.], [0., -20.], 
								[0., 20.], [-80., 20.]])

	# Ugly
	# outer_boundary = np.array([[80., 140.], [120., 100.], [80., 100.], [20., 140.], 
	# 							[40., 60.], [-40., 40.], [80., -20.], [80., 80.], 
	# 							[160., -20.], [140., 80.], [300., 20.], [300., 120.], 
	# 							[260., 60.], [180., 80.], [160., 180.], [320., 160.], 
	# 							[140., 220.], [140., 120.]])

	# # Six
	# outer_boundary = np.array([[-80., 130.], [-80., 10.], [20., 10.], [20., 80.], 
	# 							[-70., 80.], [-70., 120.], [20., 120.], [20., 130.]])

	# Weird E letter
	# outer_boundary = np.array([[-90., 120.], [-90., 10.], [-10., 10.], [-10., 20.], 
	# 							[-80., 20.], [-80., 40.], [-10., 40.], [-10., 60.], 
	# 							[-80., 50.], [-80., 70.], [-30., 60.], [-20., 80.], 
	# 							[-80., 80.], [-80., 90.], [-10., 90.], [-10., 100.], 
	# 							[-80., 100.], [-80., 110.], [-10., 110.], [-10., 120.]])

	# Office
	# outer_boundary = np.array([[-100., 90.], [-100., 70.], [-70., 70.], [-70., 64.], 
	# 							[-100., 64.], [-100., 26.], [-72., 26.], [-72., 38.], 
	# 							[-66., 38.], [-66., 22.], [-100., 22.], [-100., -70.], 
	# 							[-80., -70.], [-80., -22.], [-74., -22.], [-74., -48.], 
	# 							[-54., -48.], [-54., -54.], [-74., -54.], [-74., -70.], 
	# 							[-20., -70.], [-20., 10.], [-60., 10.], [-60., 14.], 
	# 							[-20., 14.], [-20., 52.], [-50., 52.], [-50., 56.], 
	# 							[-20., 56.], [-20., 90.], [-56., 90.], [-56., 74.], 
	# 							[-60., 74.], [-60., 90.]])

	# outer_boundary = np.array([[80., 80.], [-80., 80.], [-80., -80.], [80., -80.]])
	holes = {
			 # "obs1": np.array([[-60., 60.], [-20., 60.], [-20., 20.], [-60., 20.]]), 
			 # "obs2": np.array([[20., 60.], [60., 60.], [60., 20.], [20., 20.]]),
			 # "obs3": np.array([[20., -20.], [60., -20.], [60., -60.], [20., -60.]]),
			 # "obs4": np.array([[-60., -20.], [-20., -20.], [-20., -60.], [-60., -60.]]),
			 # "obs5": np.array([[-10., 10.], [10., 10.], [10., -10.], [-10., -10.]])
			}

	poly = sg.Polygon(outer_boundary, holes.values())
	contracted_poly = poly.buffer(- resolution / 2.)

	xmin, ymin, xmax, ymax = poly.bounds
	print("Bounds - Min: ({0}, {1}), Max: ({2}, {3})".format(xmin, ymin, xmax, ymax))

	xidx = xmin
	valid_metric_points = []

	while xidx < xmax:
		yidx = ymin

		while yidx < ymax:
			p = sg.Point(xidx, yidx)

			if contracted_poly.contains(p):
				valid_metric_points.append((xidx, yidx))

			yidx += resolution
		xidx += resolution

	bfs_agents = dict()
	# np.random.seed(100)
	rand_positions = get_random_positions(contracted_poly, (xmin, ymin, xmax, ymax), 
										  agent_count, separation=5.)

	metric_partition = dict([(i, []) for i in range(1, agent_count + 1)])
	metric_assignment = dict()

	for i in range(1, agent_count + 1):
		rp_in_gp = np.array(valid_metric_points[0])
		min_gp_dist = np.linalg.norm(rp_in_gp - rand_positions[i - 1])

		for j in range(1, len(valid_metric_points)):
			v = np.array(valid_metric_points[j])
			dist = np.linalg.norm(v - rand_positions[i - 1])
			if dist < min_gp_dist:
				min_gp_dist = dist
				rp_in_gp = v

		bfs_agents[i] = BFSAgent(i, rand_positions[i - 1], rp_in_gp, resolution)

	comp_start = time.time()
	while not check_emptiness(bfs_agents):
		deletion_updates = set()

		for k, agent in bfs_agents.items():
			expansions = agent.frontier_expand()

			if expansions is None:
				continue

			for expansion in expansions:
				if metric_assignment.get(expansion) is None:
					metric_assignment[expansion] = k

				else:
					if metric_assignment[expansion] != k:
						deletion_updates.add((expansion, k))

		for i in range(1, agent_count + 1):
			f_i = bfs_agents[i].frontier

			for j in range(i + 1, agent_count + 1):
				f_j = bfs_agents[j].frontier

				f_intersection = f_i & f_j

				for f_vmp in f_intersection:
					deletion_updates.add((f_vmp, i))
					deletion_updates.add((f_vmp, j))

		for d_upd in deletion_updates:
			try:
				bfs_agents[d_upd[1]].frontier.remove(d_upd[0])
			except Exception as e:
				pass

	elapsed = time.time() - comp_start
	print("Elapsed: {}".format(elapsed))

	for vmp, aid in metric_assignment.items():
		metric_partition[aid].append(vmp)

	figure = plt.figure()
	ax = figure.add_subplot(1, 1, 1)

	ax.clear()
	ax.set_aspect("equal")
	ax.set_title("Metric Graph")
	ax.set_xlim(xmin - 5., xmax + 5.)
	ax.set_ylim(ymin - 5., ymax + 5.)

	ax.add_patch(plt.Polygon(outer_boundary, fill=False, color=(0., 0., 1.)))

	for _, hole in holes.items():
		ax.add_patch(plt.Polygon(hole, fill=True, color=(1., 0., 0.), alpha=0.2))

	# xms, yms = zip(*valid_metric_points)
	# ax.scatter(xms, yms, s=4, c=(0., 1., 0.))

	for i in range(len(rand_positions)):
		ax.add_artist(plt.Circle(tuple(rand_positions[i]), 2., color=__COLORS[i + 1]))

	for k, partition in metric_partition.items():
		if len(partition) == 0:
			print("[WARNING] Partition for {} is empty!".format(k))
			continue

		print("Agent {} - Partition size: {}".format(k, len(partition)))
		mpx, mpy = zip(*partition)
		ax.scatter(mpx, mpy, s=8, color=__COLORS[k])

		# center = np.mean(partition, axis=0)
		# ax.add_artist(plt.Circle(tuple(center), 2., color=__COLORS[k], alpha=0.5))

	plt.show()
