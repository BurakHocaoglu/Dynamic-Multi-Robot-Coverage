#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import json
import rospy
import signal
import datetime
import traceback
import threading

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
import shapely.geometry as shgeom

import skgeom
from skgeom import boolean_set

from scipy.spatial import ConvexHull
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import floyd_warshall

from collections import deque
from functools import partial
from queue import PriorityQueue

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.5,0.5,0.5), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.99,0.99,0)]

valid_actions = [
	(np.array((1., 0.)), 1.),
	(np.array((1., 1.)), np.sqrt(2)),
	(np.array((0., 1.)), 1.),
	(np.array((-1., 1.)), np.sqrt(2)),
	(np.array((-1., 0.)), 1.),
	(np.array((-1., -1.)), np.sqrt(2)),
	(np.array((0., -1.)), 1.),
	(np.array((1., -1.)), np.sqrt(2))
]

all_agents = dict()
all_states = dict()
all_partitions = dict()
all_subregions = dict()
motion_history = dict()

exp_region = []
region_patch = None
exp_obstacles = dict()
obstacle_patches = dict()

experiment_stop = False
experiment_configuration = dict()
experiment_thread = None

environment = None
final_view_drawn = False
final_view_lock = threading.Lock()

debug_intersections = []


def from_point_to_segment(point, p_start, p_end):
	A = point[0] - p_start[0]
	B = point[1] - p_start[1]
	C = p_end[0] - p_start[0]
	D = p_end[1] - p_start[1]

	dp = A * C + B * D
	len_sq = C * C + D * D
	param = -1

	if len_sq > 0.001:
		param = dp / len_sq

	if param < 0:
		return np.array(p_start)

	elif param > 1:
		return np.array(p_end)

	else:
		return np.array([p_start[0] + param * C, p_start[1] + param * D])

def get_closest_point_in_env(env, target):
	min_dist = 0
	target_segment = None

	for i in range(len(env.outer_boundary)):
		j = (i + 1) % len(env.outer_boundary)
		p_i = env.outer_boundary[i]
		p_j = env.outer_boundary[j]

		P = from_point_to_segment(target, p_i, p_j)
		d = np.linalg.norm(target - P)

		if d < min_dist:
			target_segment = (i, j, P)
			min_dist = d

	for _, hole in env.holes.items():
		for i in range(len(hole)):
			j = (i + 1) % len(hole)
			p_i = env.outer_boundary[i]
			p_j = env.outer_boundary[j]

			P = from_point_to_segment(target, p_i, p_j)
			d = np.linalg.norm(target - P)

			if d < min_dist:
				target_segment = (i, j, P)
				min_dist = d

	if target_segment is None:
		return target

	return target_segment[2]

def seg_in_pwh(seg, pwh):
	do_intersect = False

	for i in range(len(pwh.outer_boundary().coords)):
		j = (i + 1) % len(pwh.outer_boundary().coords)

		work_seg = skgeom.Segment2(
			skgeom.Point2(pwh.outer_boundary().coords[i][0], pwh.outer_boundary().coords[i][1]), 
			skgeom.Point2(pwh.outer_boundary().coords[j][0], pwh.outer_boundary().coords[j][1]))

		S_int = skgeom.intersection(seg, work_seg)
		if S_int is not None:
			do_intersect = True
			break

	return do_intersect

def angle_in_2pi(v):
	return np.arctan2(v[1], v[0])

def angular_sort(reference, vertices):
	# print(vertices)
	vectors = [p - reference for p in vertices]
	indexed_angles = [(angle_in_2pi(vectors[i]), i) for i in range(len(vectors))]
	indexed_angles.sort()
	return [vertices[i] for _, i in indexed_angles]

def inside_check(polyWHole, p):
	pnt = skgeom.Point2(p[0], p[1])
	if polyWHole.outer_boundary().oriented_side(pnt) != skgeom.Sign.POSITIVE:
		return False

	for h in polyWHole.holes:
		if h.oriented_side(pnt) != skgeom.Sign.POSITIVE:
			return False

	return True

def get_subgraph_eccentricity(G, target, deleted):
	subgraph_nodes = dict()
	N = G.shape[0]
	sN = 0
	Q = deque()
	Q.append(target)
	subgraph_nodes[target] = sN
	visited = dict()
	visited[target] = True
	sN += 1
	edges = []

	while len(Q) > 0:
		node = Q.popleft()

		for i in range(N):
			v_node = visited.get(i)
			# print(node, i)

			if i == target or i == deleted:
				continue

			elif G[node, i] != 0:
				if v_node is None or not v_node:
					Q.append(i)
					visited[i] = True
					subgraph_nodes[i] = sN
					edges.append((node, i))
					sN += 1

	subgraph = np.zeros((sN, sN))
	for (n1, n2) in edges:
		sId1 = subgraph_nodes[n1]
		sId2 = subgraph_nodes[n2]
		subgraph[sId1, sId2] = G[n1, n2]
		subgraph[sId2, sId1] = G[n2, n1]

	# print(subgraph)
	dists, preds = floyd_warshall(csgraph=subgraph, directed=False, return_predecessors=True)
	# dists[dists == np.inf] = 0
	max_shortest_paths = np.amax(dists, axis=0)
	# print(max_shortest_paths)
	# geodesic_cand_idx = np.argmin(total_shortest_paths)
	return min(max_shortest_paths)


class BFSAgent:

	def __init__(self, aid, pos, gpos, step_size, mode="CD"):
		self.id = aid
		self.p = pos
		self.gp = gpos
		self.step_size = step_size
		self.mode = mode

		self.work = 0.
		self.actual_work = 0.
		self.max_geod_dist = 0.
		self.alive = True

		self.limited_sensing = False
		self.sense_radius = 10000.
		self.physical_radius = 0.5
		# ...

		self.visited = set()
		self.frontier = set()
		self.parents = dict()
		self.borders = dict()
		self.edges = dict()
		self.dvalue = 0

		self.frontier.add(tuple(self.gp))
		self.visited.add(tuple(self.gp))
		self.parents[tuple(self.gp)] = None
		self.normals = dict()
		self.orphan_border_vertices = []

		self.cvx_voronoi_cell = None
		self.work_geometry = None
		self.bnd_tolerance = 10.

		self.convergence_count = 0
		self.goal = np.zeros(2)
		self.valid_segments = set()

	def set_behaviours(self, settings=dict()):
		self.limited_sensing = False if settings.get("limited_sensing") is None else settings.get("limited_sensing")
		self.sense_radius = 10000. if settings.get("sense_radius") is None else settings.get("sense_radius")
		self.physical_radius = 0.5 if settings.get("physical_radius") is None else settings.get("physical_radius")
		# ...

	def reset(self):
		self.valid_segments = set()
		self.work = 0.
		self.actual_work = 0.
		self.max_geod_dist = 0.

		self.visited = set()
		self.frontier = set()
		self.parents = dict()
		self.borders = dict()
		self.edges = dict()
		self.dvalue = 0

		self.frontier.add(tuple(self.gp))
		self.visited.add(tuple(self.gp))
		self.parents[tuple(self.gp)] = None
		self.normals = dict()
		self.orphan_border_vertices = []

	def update(self, partition):
		displacement = np.zeros(2)
		coeff = 0.1

		if partition is not None:
			if self.mode == "CD":
				coeff = 1.
				index = 0
				location_map = dict()
				index_map = dict()
				for p in partition:
					location_map[p] = index
					index_map[index] = p
					index += 1

				displacement = self.geodesic_force(location_map, index_map)



				disp_x, disp_y = np.array([displacement[0], 0.]), np.array([0., displacement[1]])
				p_next = self.p + displacement
				p_next_valid = ((globals()["environment"] is not None) and 
								(globals()["environment"].is_point_valid(p_next, False)))

				if not p_next_valid:
					p_next_in_x = self.p + disp_x
					p_next_in_x_valid = ((globals()["environment"] is not None) and 
										 (globals()["environment"].is_point_valid(p_next_in_x, False)))

					p_next_in_y = self.p + disp_y
					p_next_in_y_valid = ((globals()["environment"] is not None) and 
										 (globals()["environment"].is_point_valid(p_next_in_y, False)))

					if p_next_in_x_valid and p_next_in_y_valid:
						if np.linalg.norm(disp_x) >= np.linalg.norm(disp_y):
							displacement = disp_x

						else:
							displacement = disp_y

					elif p_next_in_x_valid and not p_next_in_y_valid:
						displacement = disp_x

					elif not p_next_in_x_valid and p_next_in_y_valid:
						displacement = disp_y

					else:
						displacement = np.zeros(2)



			elif self.mode == "GA":
				coeff = 0.5
				self.work_geometry = partition

				if partition.outer_boundary().orientation() == skgeom.Sign.CLOCKWISE:
					partition.outer_boundary().reverse_orientation()

				skel = skgeom.skeleton.create_interior_straight_skeleton(partition)

				displacement = self.skeletal_force(skel)
				# displacement *= self.step_size / np.linalg.norm(displacement)

				# cm = self.compute_next_wp_to_geometric_center(work_cell)
				# displacement = cm - self.p

			elif self.mode == "G":
				coeff = 0.5
				self.max_geod_dist = 0. # NEEDS UPDATE

				self.work_geometry = partition

				# if self.id == 0:
				# 	print("Agent {}:\n{}".format(self.id, self.work_geometry))
				# 	print("---")

				cm = Environment.compute_geometric_center(partition)
				cm_valid = ((globals()["environment"] is not None) and 
							(globals()["environment"].is_point_valid(cm, False)))

				if cm_valid:
					displacement = self.tangent_bug(cm)

				else:
					if globals()["environment"] is None:
						return

					p_nearest = get_closest_point_in_env(globals()["environment"], cm)

					# displacement = self.tangent_bug(p_nearest)

					displacement = p_nearest - self.p

					disp_x, disp_y = np.array([displacement[0], 0.]), np.array([0., displacement[1]])
					p_next = self.p + displacement * 0.1
					p_next_valid = ((globals()["environment"] is not None) and 
									(globals()["environment"].is_point_valid(p_next, False)))

					if not p_next_valid:
						p_next_in_x = self.p + disp_x
						p_next_in_x_valid = ((globals()["environment"] is not None) and 
											 (globals()["environment"].is_point_valid(p_next_in_x, False)))

						p_next_in_y = self.p + disp_y
						p_next_in_y_valid = ((globals()["environment"] is not None) and 
											 (globals()["environment"].is_point_valid(p_next_in_y, False)))

						if p_next_in_x_valid and p_next_in_y_valid:
							if np.linalg.norm(disp_x) >= np.linalg.norm(disp_y):
								displacement = disp_x

							else:
								displacement = disp_y

						elif p_next_in_x_valid and not p_next_in_y_valid:
							displacement = disp_x

						elif not p_next_in_x_valid and p_next_in_y_valid:
							displacement = disp_y

						else:
							displacement = np.zeros(2)

			else:
				print("Agent {} unknown mode! Staying still...".format(self.id))
				pass

		else:
			print("Agent {} - No work!".format(self.id))

		displacement /= np.linalg.norm(displacement)
		# displacement *= self.step_size / np.linalg.norm(displacement)

		p_next = self.p + displacement * coeff
		# self.goal = p_next
		p_next_valid = ((globals()["environment"] is not None) and 
						(globals()["environment"].is_point_valid(p_next, False)))

		# self.p = p_next
		# self.gp = np.array(np.round(self.p / self.step_size) * self.step_size)

		if p_next_valid:
			self.p = p_next
			self.gp = np.array(np.round(self.p / self.step_size) * self.step_size)

		else:
			# print("Agent {} outside move!".format(self.id))
			pass

	def tangent_bug(self, goal):
		force = np.zeros(2)

		arr = skgeom.arrangement.Arrangement()

		for i in range(len(globals()["environment"].outer_boundary)):
			j = (i + 1) % len(globals()["environment"].outer_boundary)

			arr.insert(skgeom.Segment2(
				skgeom.Point2(
					globals()["environment"].outer_boundary[i][0], 
					globals()["environment"].outer_boundary[i][1]), 
				skgeom.Point2(
					globals()["environment"].outer_boundary[j][0], 
					globals()["environment"].outer_boundary[j][1])
				)
			)

		for _, hole in globals()["environment"].holes.items():
			for i in range(len(hole)):
				j = (i + 1) % len(hole)
				arr.insert(skgeom.Segment2(skgeom.Point2(hole[i][0], hole[i][1]), skgeom.Point2(hole[j][0], hole[j][1])))

		vis_pivot = skgeom.Point2(self.p[0], self.p[1])
		vis_face = arr.find(vis_pivot)
		VS = skgeom.RotationalSweepVisibility(arr)
		vis = VS.compute_visibility(vis_pivot, vis_face)

		vertex_id = 0
		vertices = dict()
		V_vis_poly = []
		for v in vis.halfedges:
			vert = (float(v.target().point().x()), float(v.target().point().y()))

			if vertices.get(vert) is None:
				vertices[vertex_id] = vert
				vertex_id += 1
				V_vis_poly.append(vert)

		VIS = skgeom.Polygon(V_vis_poly)

		try:
			if VIS.oriented_side(skgeom.Point2(goal[0], goal[1])) != skgeom.Sign.NEGATIVE:
				force = goal - self.p

			else:
				found = False

				intersections = []
				seg_direct = skgeom.Segment2(
					skgeom.Point2(self.p[0], self.p[1]), 
					skgeom.Point2(goal[0], goal[1])
				)

				for i in range(len(globals()["environment"].outer_boundary)):
					j = (i + 1) % len(globals()["environment"].outer_boundary)

					bSeg = skgeom.Segment2(
						skgeom.Point2(
							globals()["environment"].outer_boundary[i][0], 
							globals()["environment"].outer_boundary[i][1]), 
						skgeom.Point2(
							globals()["environment"].outer_boundary[j][0], 
							globals()["environment"].outer_boundary[j][1])
					)

					P_int = skgeom.intersection(seg_direct, bSeg)
					if P_int is not None:
						intersections.append((i, j, P_int))

				# if self.id == 0:
				# 	print("Agent {} - I: {}".format(self.id, len(intersections)))

				if len(intersections) == 0:
					force = np.zeros(2)

				else:
					indices = list(range(intersections[0][0], intersections[-1][0] + 1))

					start_segment, end_segment = -1, -1
					d_min_start, d_min_end = -1, -1

					# for idx in indices:
					for (idx0, idx1, _) in intersections:

						s_i = from_point_to_segment(
							self.p, 
							globals()["environment"].outer_boundary[idx0], 
							globals()["environment"].outer_boundary[idx1]
						)

						e_i = from_point_to_segment(
							goal, 
							globals()["environment"].outer_boundary[idx0], 
							globals()["environment"].outer_boundary[idx1]
						)

						if d_min_start < 0 or d_min_start > np.linalg.norm(s_i - self.p):
							start_segment = idx0
							d_min_start = np.linalg.norm(s_i - self.p)

						if d_min_end < 0 or d_min_end > np.linalg.norm(e_i - goal):
							end_segment = idx0
							d_min_end = np.linalg.norm(e_i - goal)

					direction = 1 if start_segment < end_segment else -1

					min_idx = 0
					d_min = -1
					for bidx in range(start_segment, end_segment + direction, direction):
						p_target = globals()["environment"].outer_boundary[bidx]

						if VIS.oriented_side(skgeom.Point2(p_target[0], p_target[1])) != skgeom.Sign.NEGATIVE:
							d_target = p_target - self.p

							if d_min < 0 or np.linalg.norm(d_target) < d_min:
								min_idx = bidx
								d_min = np.linalg.norm(d_target)

							found = True

					if not found:
						force = np.zeros(2)

					else:
						force = globals()["environment"].outer_boundary[min_idx] - self.p

		except Exception as e:
			print("ERROR - Agent {}:".format(self.id))
			print(traceback.format_exc())
			raise e

		return force

	def frontier_expand(self):
		if globals()["environment"] is None:
			return None

		if len(self.frontier) == 0:
			return None

		next_wave = set()

		while len(self.frontier):
			f_pos = self.frontier.pop()

			if self.edges.get(f_pos) is None:
				self.edges[f_pos] = set()

			relative_expansion = []
			env_limited = False
			for act in globals()["valid_actions"]:
				move, cost = act[0] * self.step_size, act[1] * self.step_size

				next_pos = np.array(f_pos) + move
				if not globals()["environment"].is_point_valid(next_pos):
					env_limited = True
					continue

				if tuple(next_pos) not in self.edges[f_pos]:
					self.edges[f_pos].add(tuple(next_pos))

					if self.edges.get(tuple(next_pos)) is None:
						self.edges[tuple(next_pos)] = set()

					self.edges[tuple(next_pos)].add(f_pos)

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

	def voronoi_subregion(self):
		neighbour_segments = []
		constraints = []
		values = []

		self.valid_segments = set()

		for aid, state in globals()["all_states"].items():
			if aid == self.id:
				continue

			if not state["alive"]:
				continue

			r_ij = state["pos"] - self.p
			m_ij = (state["pos"] + self.p) * 0.5
			norm = np.linalg.norm(r_ij)

			if self.limited_sensing:
				if norm <= self.sense_radius:
					neighbour_segments.append((r_ij, m_ij, aid))
					constraints.append(r_ij)
					values.append(np.dot(r_ij, m_ij) - self.physical_radius)

			else:
				neighbour_segments.append((r_ij, m_ij, aid))
				constraints.append(r_ij)
				values.append(np.dot(r_ij, m_ij) - self.physical_radius)

		b = np.array(values)
		A = np.array(constraints)

		cvx_voronoi_vertices = []
		for i in range(len(neighbour_segments) - 1):
			n_i, m_i, a_i = neighbour_segments[i]
			d_i = np.dot(n_i, m_i)

			for j in range(i + 1, len(neighbour_segments)):
				n_j, m_j, a_j = neighbour_segments[j]
				d_j = np.dot(n_j, m_j)

				try:
					A_intr = np.array([n_i.round(2), n_j.round(2)])
					b_intr = np.array([d_i.round(2), d_j.round(2)])
					p_intr = np.linalg.solve(A_intr, b_intr).round(2)

				except np.linalg.LinAlgError:
					continue

				except Exception as e:
					print(traceback.format_exc())
					continue

				# if self.id == 0:
				# 	globals()["debug_intersections"].append(p_intr)

				# if not globals()["environment"].is_point_valid(p_intr):
				if not globals()["environment"].is_point_valid(p_intr, True):
					continue

				if np.all(A.dot(p_intr) <= b + self.bnd_tolerance):
					cvx_voronoi_vertices.append(p_intr)
					self.valid_segments.add((tuple(n_i), tuple(m_i), a_i))
					self.valid_segments.add((tuple(n_j), tuple(m_j), a_j))

		to_avoid = globals()["environment"].segments_to_avoid
		for i in range(len(neighbour_segments)):
			n_i, m_i, _ = neighbour_segments[i]
			d_i = np.dot(n_i, m_i)

			for j in range(len(to_avoid)):
				n_j, m_j = to_avoid[j]
				d_j = np.dot(n_j, m_j)

				try:
					A_intr = np.array([n_i.round(2), n_j.round(2)])
					b_intr = np.array([d_i.round(2), d_j.round(2)])
					p_intr = np.linalg.solve(A_intr, b_intr).round(2)

				except np.linalg.LinAlgError:
					continue

				except Exception as e:
					print(traceback.format_exc())
					continue

				# if self.id == 0:
				# 	globals()["debug_intersections"].append(p_intr)

				# if not globals()["environment"].is_point_valid(p_intr):
				if not globals()["environment"].is_point_valid(p_intr, True):
					# if self.id == 0:
					# 	print("Agent {} - {} OUTSIDE".format(self.id, p_intr))

					continue

				if np.all(A.dot(p_intr) <= b + self.bnd_tolerance):
					cvx_voronoi_vertices.append(p_intr)

				# else:
				# 	if self.id == 0:
				# 		print("Agent {} [v {}-{}] - {} INFEASIBLE".format(self.id, i + 1, j + 1, p_intr))

		# for v in globals()["environment"].outer_boundary:
		for i in range(len(globals()["environment"].hull_x)):
			v = np.array([globals()["environment"].hull_x[i], 
						  globals()["environment"].hull_y[i]])

			if len(neighbour_segments) > 0:
				product = A.dot(v) <= b + self.bnd_tolerance
				feasibility = np.all(product)

				if feasibility:
					cvx_voronoi_vertices.append(v)

				# if self.id == 0:
				# 	# globals()["debug_intersections"].append(p_intr)
				# 	print("Agent {} - {} B - INFEASIBLE".format(self.id, v))

			else:
				cvx_voronoi_vertices.append(v)


		cvx_sorted = angular_sort(self.p, cvx_voronoi_vertices)
		if len(cvx_sorted) == 0:
			return None

		pruned = [cvx_sorted[0]]

		i, k = 1, 0
		nV = len(cvx_sorted)
		while i < nV:
			if np.linalg.norm(cvx_sorted[i] - cvx_sorted[k]) >= self.physical_radius:
				pruned.append(cvx_sorted[i])
				k = i
			i += 1

		self.cvx_voronoi_cell = skgeom.Polygon(pruned)
		if self.cvx_voronoi_cell.orientation() == skgeom.Sign.CLOCKWISE:
			self.cvx_voronoi_cell.reverse_orientation()

		work_cell = self.cvx_voronoi_cell

		if not isinstance(work_cell, skgeom.PolygonWithHoles):
			return skgeom.PolygonWithHoles(work_cell, [])

		else:
			return work_cell

	def skeletal_force(self, skel):
		force = np.zeros(2)

		N = 0
		vertexIds = dict()
		indexMap = dict()
		borderPairs = []
		edgeList = []

		force_work_diff = np.zeros(2)

		for h in skel.halfedges:
			p1 = h.vertex.point
			p2 = h.opposite.vertex.point
			np1 = np.array((p1.x(), p1.y()), dtype=float)
			np2 = np.array((p2.x(), p2.y()), dtype=float)

			if vertexIds.get(tuple(np1)) is None:
				vertexIds[tuple(np1)] = N
				indexMap[N] = np1
				N += 1

			if vertexIds.get(tuple(np2)) is None:
				vertexIds[tuple(np2)] = N
				indexMap[N] = np2
				N += 1

			# if h.is_border:
			# 	for seg in self.valid_segments:
			# 		seg_n, seg_m, seg_id = seg
			# 		dp1 = np.absolute(np.dot(np1 - seg_m, seg_n)) / np.linalg.norm(seg_n)
			# 		dp2 = np.absolute(np.dot(np2 - seg_m, seg_n)) / np.linalg.norm(seg_n)

			# 		if dp1 <= self.physical_radius and dp2 <= self.physical_radius:
			# 			if self.work == 0:
			# 				print("Agent {} has 0 work for GA (vs. {} for {})!".format(self.id, globals()["all_agents"][seg_id].work, seg_id))

			# 			# shared_seg = skgeom.Segment2(
			# 			# 	skgeom.Point2(np1[0], np1[1]), 
			# 			# 	skgeom.Point2(np2[0], np2[1]))

			# 			# nb_area = skgeom.PolygonWithHoles(
			# 			# 	globals()["all_subregions"][seg_id]["poly"],
			# 			# 	globals()["all_subregions"][seg_id]["holes"])

			# 			# work_diff = 0.
			# 			# if not seg_in_pwh(shared_seg, nb_area):
			# 			# 	work_diff = self.work
			# 			# 	# work_diff = - self.work ** 2
			# 			# 	pass

			# 			# else:
			# 			# 	work_diff = globals()["all_agents"][seg_id].work - self.work
			# 			# 	# work_diff = globals()["all_agents"][seg_id].work ** 2 - self.work ** 2

			# 			work_diff = globals()["all_agents"][seg_id].work - self.work
			# 			# work_diff = globals()["all_agents"][seg_id].work ** 2 - self.work ** 2

			# 			borderPairs.append((vertexIds[tuple(np1)], vertexIds[tuple(np2)], seg_id, work_diff))

			# elif h.is_bisector:
			if h.is_bisector:
				np1Id = vertexIds[tuple(np1)]
				np2Id = vertexIds[tuple(np2)]
				edgeList.append((np1Id, np2Id, np1, np2))

		graph = np.zeros((N, N))

		for edge in edgeList:
			d = np.linalg.norm(edge[2] - edge[3])
			graph[edge[0], edge[1]] = d
			graph[edge[1], edge[0]] = d

		dists, preds = floyd_warshall(csgraph=graph, directed=False, return_predecessors=True)
		dists[dists == np.inf] = 0
		# total_shortest_paths = np.sum(dists, axis=0)
		# # geodesic_cand_idx = np.argmin(total_shortest_paths)
		# if len(total_shortest_paths) < 2:
		# 	print("Agent {} - Needs more skeleton vertices ({})!".format(self.id, len(total_shortest_paths)))
		# 	return np.zeros(2)

		# ecc_sorted = np.argsort(total_shortest_paths)
		# cand_1, cand_2 = ecc_sorted[:2]

		# if graph[cand_1, cand_2] == 0 or graph[cand_2, cand_1] == 0:
		# 	print("Agent {} - Candidates are not neighbours (F: {} - B: {}) !".format(
		# 		self.id, graph[cand_1, cand_2], graph[cand_2, cand_1]))
		# 	return force

		# ecc_1 = get_subgraph_eccentricity(graph, cand_1, cand_2)
		# ecc_2 = get_subgraph_eccentricity(graph, cand_2, cand_1)

		if N < 2:
			return force

		cand_1, cand_2 = 0, 1
		min_ecc_1, min_ecc_2 = 1e9, 1e9
		for i in range(N - 1):
			for j in range(i + 1, N):
				ecc_1 = get_subgraph_eccentricity(graph, i, j)
				ecc_2 = get_subgraph_eccentricity(graph, j, i)
				pass

				if ecc_1 < min_ecc_1 and ecc_2 < min_ecc_2:
					cand_1 = i
					cand_2 = j

		# additive_avg = (indexMap[cand_1] + indexMap[cand_2]) * 0.5
		# additive_avg = ((indexMap[cand_1] * total_shortest_paths[cand_2] + 
		# 				 indexMap[cand_2] * total_shortest_paths[cand_1]) / 
		# 				(total_shortest_paths[cand_1] + total_shortest_paths[cand_2]))
		# additive_avg = indexMap[cand_1]
		additive_avg = ((indexMap[cand_1] * ecc_1 + indexMap[cand_2] * ecc_2) / (ecc_1 + ecc_2))

		approx_geod_center_force = np.zeros(2)

		heuristics = np.array([np.linalg.norm(self.p - indexMap[i]) for i in range(N)])
		root_id = np.argmin(heuristics)

		arr = skgeom.arrangement.Arrangement()

		for i in range(len(globals()["environment"].outer_boundary)):
			j = (i + 1) % len(globals()["environment"].outer_boundary)

			arr.insert(skgeom.Segment2(
				skgeom.Point2(
					globals()["environment"].outer_boundary[i][0], 
					globals()["environment"].outer_boundary[i][1]), 
				skgeom.Point2(
					globals()["environment"].outer_boundary[j][0], 
					globals()["environment"].outer_boundary[j][1])
				)
			)

		for _, hole in globals()["environment"].holes.items():
			for i in range(len(hole)):
				j = (i + 1) % len(hole)
				arr.insert(skgeom.Segment2(skgeom.Point2(hole[i][0], hole[i][1]), skgeom.Point2(hole[j][0], hole[j][1])))

		vis_pivot = skgeom.Point2(self.p[0], self.p[1])
		vis_face = arr.find(vis_pivot)
		VS = skgeom.RotationalSweepVisibility(arr)
		vis = VS.compute_visibility(vis_pivot, vis_face)

		vertex_id = 0
		vertices = dict()
		V_vis_poly = []
		for v in vis.halfedges:
			vert = (float(v.target().point().x()), float(v.target().point().y()))

			if vertices.get(vert) is None:
				vertices[vertex_id] = vert
				vertex_id += 1
				V_vis_poly.append(vert)

		VIS = skgeom.Polygon(V_vis_poly)

		try:
			# --------------------------------------------------------------------------

			if VIS.oriented_side(skgeom.Point2(additive_avg[0], additive_avg[1])) != skgeom.Sign.NEGATIVE:
				approx_geod_center_force = additive_avg - self.p

			else:
				prev_id = cand_1 if min_ecc_1 < min_ecc_2 else cand_2
				loc_id = prev_id

				while loc_id != root_id and loc_id != -9999: #and loc_id != prev_id:
					prev_id = loc_id
					loc_id = preds[root_id, loc_id]

				self.goal = np.array(indexMap[prev_id])
				approx_geod_center_force = np.array(indexMap[prev_id]) - self.gp

				# k = 0
				# while k < len(ecc_sorted):
				# 	pos = indexMap[ecc_sorted[k]]

				# 	if VIS.oriented_side(skgeom.Point2(pos[0], pos[1])) != skgeom.Sign.NEGATIVE:
				# 		break

				# 	k += 1

				# if k < len(ecc_sorted):
				# 	approx_geod_center_force = np.array(indexMap[k]) - self.p

				# else:
				# 	print("Agent {} - outsizing!".format(self.id))

			# for (vId1, vId2, nbId, dW) in borderPairs:
			# 	# Parent vertex in the skeleton
			# 	d_vp1, i_vp1 = dists[vId1, 0], 0
			# 	d_vp2, i_vp2 = dists[vId2, 0], 0
			# 	for u in range(1, N):
			# 		if u != vId1 and dists[vId1, u] < d_vp1:
			# 			d_vp1 = dists[vId1, u]
			# 			i_vp1 = u

			# 		if u != vId2 and dists[vId2, u] < d_vp2:
			# 			d_vp2 = dists[vId2, u]
			# 			i_vp2 = u

			# 	vp1 = indexMap[i_vp1]
			# 	vp2 = indexMap[i_vp2]
			# 	vp = vp1

			# 	if vertexIds[tuple(vp1)] != vertexIds[tuple(vp2)]:
			# 		w_norm_1 = np.linalg.norm(globals()["all_agents"][nbId].p - vp1)
			# 		w_norm_2 = np.linalg.norm(globals()["all_agents"][nbId].p - vp2)
			# 		if w_norm_1 > w_norm_2:
			# 			vp = vp2

			# 	np1 = indexMap[vId1]
			# 	np2 = indexMap[vId2]
			# 	mid = (np1 + np2) * 0.5
			# 	# v_dir = np2 - np1
			# 	# v_norm = np.array([-v_dir[1], v_dir[0]])
			# 	dW_i = vp - mid
			# 	dW_i /= np.linalg.norm(dW_i)

			# 	prev_id = vertexIds[tuple(vp)]
			# 	loc_id = vertexIds[tuple(vp)]
			# 	# prev_id = vertexIds[vp]
			# 	# loc_id = vertexIds[vp]

			# 	while loc_id != root_id and loc_id != -9999: #and loc_id != prev_id:
			# 		prev_id = loc_id
			# 		loc_id = preds[root_id, loc_id]

			# 	if dW < 0:
			# 		# Neighbour has less work
			# 		force_work_diff += self.p - np.array(indexMap[prev_id]) - dW_i

			# 	elif dW > 0:
			# 		# Neighbour has more work
			# 		force_work_diff += np.array(indexMap[prev_id]) - self.p + dW_i

			# 	else:
			# 		# Neighbour has the same work
			# 		# force_work_diff = np.zeros(2)
			# 		pass

		except Exception as e:
			print("ERROR - Agent {}:".format(self.id))
			# print("---")
			# print(locations, tuple(self.gp) in locations)
			print("---")
			print(traceback.format_exc())
			print("---")
			raise e

		# force_work_diff *= 5. / np.linalg.norm(force_work_diff)

		force += approx_geod_center_force
		# force += force_work_diff
		# if self.id == 0:
		# 	print("Agent {} - G: {} | W: {}".format(self.id, 
		# 		approx_geod_center_force, force_work_diff))

		# force += globals()["experiment_configuration"]["K_att"] * approx_geod_center_force
		# if self.id == 0:
		# 		print("Agent 0 - F_agc: {}, F_wd: {}, F_net: {}".format(
		# 			np.linalg.norm(approx_geod_center_force), 
		# 			np.linalg.norm(force_work_diff), 
		# 			force))

		# force *= self.step_size / np.linalg.norm(force)

		return force

	def geodesic_force(self, locations, indices):
		force = np.zeros(2)

		N = len(locations)
		graph = np.zeros((N, N))

		for gp, gp_id in locations.items():
			v_gp = np.array(gp)

			for gp_np in self.edges[gp]:
				v_np = np.array(gp_np)

				gp_np_id = locations.get(gp_np)
				if gp_np_id is not None:
					graph[gp_id, gp_np_id] = np.linalg.norm(v_gp - v_np)
					graph[gp_np_id, gp_id] = np.linalg.norm(v_gp - v_np)

		dists, preds = floyd_warshall(csgraph=graph, directed=False, return_predecessors=True)
		dists[dists == np.inf] = 0
		total_shortest_paths = np.sum(dists, axis=0)
		geodesic_cand_idx = np.argmin(total_shortest_paths)

		approx_geod_center_force = np.zeros(2)
		workload_diff_force = np.zeros(2)

		root_id = locations[tuple(self.gp)]
		self.max_geod_dist = np.amax(dists[root_id])
		self.actual_work = np.sum(np.power(dists[root_id], 2))

		try:
			# --------------------------------------------------------------------------

			prev_id = geodesic_cand_idx 
			loc_id = geodesic_cand_idx

			while loc_id != root_id and loc_id != -9999: #and loc_id != prev_id:
				prev_id = loc_id
				loc_id = preds[root_id, loc_id]

			self.goal = np.array(indices[prev_id])
			approx_geod_center_force = np.array(indices[prev_id]) - self.gp

			# --------------------------------------------------------------------------

			work = len(locations)
			for nb_id, nb_border in self.borders.items():
				work_nb = globals()["all_agents"][nb_id].work
				d_work = work_nb - work

				for vb in nb_border:
					vb_id = locations[vb]
					prev_id = vb_id 
					loc_id = vb_id
					root_id = locations[tuple(self.gp)]

					vb_dist = dists[root_id, vb_id]

					while loc_id != root_id and loc_id != -9999: #and loc_id != prev_id:
						prev_id = loc_id
						loc_id = preds[root_id, loc_id]

					force_vb = np.array(indices[prev_id]) - self.gp
					force_vb /= np.linalg.norm(force_vb)

					if self.parents.get(vb) is None:
						continue

					v_normal = np.array(vb) - np.array(self.parents[vb])
					workload_diff_force += d_work * (vb_dist ** 2) * force_vb / np.linalg.norm(v_normal)
			# --------------------------------------------------------------------------

		except Exception as e:
			print("ERROR - Agent {}:".format(self.id))
			print("---")
			print(locations, tuple(self.gp) in locations)
			print("---")
			print(preds)
			print("---")
			print(traceback.format_exc())
			print("---")
			raise e

		workload_diff_force /= np.linalg.norm(workload_diff_force)
		# force += globals()["experiment_configuration"]["K_rep"] * workload_diff_force
		# force += globals()["experiment_configuration"]["K_att"] * approx_geod_center_force
		WD = globals()["experiment_configuration"]["K_rep"] * workload_diff_force
		GF = globals()["experiment_configuration"]["K_att"] * approx_geod_center_force
		force += WD + GF
		# force *= self.step_size / np.linalg.norm(force)

		if self.id == 0:
			non_trivial_borders = 0
			for _, nb_border in self.borders.items():
				if len(nb_border) > 0:
					non_trivial_borders += 1

			print("Agent {} - <{}> WD: {} vs GF: {} -> F: {}".format(self.id, non_trivial_borders,
				np.linalg.norm(WD), np.linalg.norm(GF), np.linalg.norm(force)))

		return force

	def compute_next_wp_to_geometric_center(self, partition):
		wp = np.zeros(2)

		pass

		return wp


class Environment:

	def __init__(self, outer_boundary=None, holes=None, resolution=1., discrete=True):
		self.outer_boundary = np.array(outer_boundary)
		self.holes = holes
		self.resolution = resolution
		self.discrete = discrete

		hull = ConvexHull(self.outer_boundary)
		self.hull_x, self.hull_y = self.outer_boundary[hull.vertices, 0], self.outer_boundary[hull.vertices, 1]

		self.segments_to_avoid = []
		for i in range(len(self.hull_x)):
			j = (i + 1) % len(self.hull_x)
			n = np.array([self.hull_y[i] - self.hull_y[j], self.hull_x[j] - self.hull_x[i]])
			m = np.array([self.hull_x[i] + self.hull_x[j], self.hull_y[i] + self.hull_y[j]]) * 0.5
			self.segments_to_avoid.append((n, m))

		self.hull_poly = skgeom.Polygon(np.array([self.hull_x, self.hull_y]).T)
		self.hull_shape = shgeom.Polygon(np.array([self.hull_x, self.hull_y]).T)

		self._bnd = shgeom.Polygon(self.outer_boundary).buffer(-1.)
		# self._bnd = shgeom.Polygon(self.outer_boundary).buffer(- self.resolution / 2.)
		self._contracted_bnd = self._bnd.buffer(- self.resolution / 2.)

		self._obs = dict()
		for obs_name, obs in self.holes.items():
			if len(obs) > 3:
				self._obs[obs_name] = shgeom.Polygon(obs).buffer(1.)
			# self._obs[obs_name] = shgeom.Polygon(obs).buffer(self.resolution / 2.)

		# self.arrangement = skgeom.arrangement.Arrangement()
		# for i in range(len(self.outer_boundary)):
		# 	j = (i + 1) % len(self.outer_boundary)
		# 	p_i, p_j = self.outer_boundary[i], self.outer_boundary[j]
		# 	seg = skgeom.Segment2(skgeom.Point2(p_i[0], p_i[1]), skgeom.Point2(p_j[0], p_j[1]))
		# 	self.arrangement.insert(seg)

		# for _, hole in self.holes.items():
		# 	for i in range(len(hole)):
		# 		j = (i + 1) & len(hole)
		# 		h_i, h_j = hole[i], hole[j]
		# 		seg = skgeom.Segment2(skgeom.Point2(h_i[0], h_i[1]), skgeom.Point2(h_j[0], h_j[1]))
		# 		self.arrangement.insert(seg)

		# self.tev = skgeom.TriangularExpansionVisibility(self.arrangement)

		sk_holes = []
		for _, hole in self.holes.items():
			if len(hole) > 3:
				sk_hole = skgeom.Polygon(hole)
				if sk_hole.orientation() == skgeom.Sign.NEGATIVE:
					sk_hole.reverse_orientation()
				sk_holes.append(sk_hole)

		self.actual_region = skgeom.PolygonWithHoles(skgeom.Polygon(self.outer_boundary), sk_holes)

		self.valid_metric_points = []
		self.limits = self._bnd.bounds
		self.global_workload = 0.
		self.n_agents = 0
		self.initialize_metric_topology()

	def initialize_metric_topology(self):
		xmin, ymin, xmax, ymax = self.limits
		xidx = xmin + self.resolution / 2.

		while xidx < xmax:
			yidx = ymin + self.resolution / 2.

			while yidx < ymax:
				p = shgeom.Point(xidx, yidx)

				if self._bnd.contains(p):
					self.valid_metric_points.append((xidx, yidx))

				yidx += self.resolution
			xidx += self.resolution

		self.global_workload = len(self.valid_metric_points)

	def is_point_valid(self, p, convex=False):
		try:
			pt = skgeom.Point2(p[0], p[1])
			if convex:
				# return self.hull_shape.contains(shgeom.Point(p))
				return self.hull_poly.oriented_side(pt) != skgeom.Sign.NEGATIVE

			else:
				if self.actual_region.outer_boundary().oriented_side(pt) == skgeom.Sign.NEGATIVE:
					return False

				for hole in self.actual_region.holes:
					# if hole.oriented_side(pt) == skgeom.Sign.POSITIVE: # Iconsistent with holes
					if hole.oriented_side(pt) != skgeom.Sign.NEGATIVE:
						return False

				return True

				# if not self._bnd.contains(shgeom.Point(p)):
				# 	return False

				# for _, o in self._obs.items():
				# 	if o.contains(shgeom.Point(p)):
				# 		return False

				# return True
		except Exception as e:
			# raise e
			return False

	def generate_random_positions(self, count):
		self.n_agents = count
		valid_samples, i = [], 0
		low_limit = [self.limits[0] + 1., self.limits[1] + 1.]
		high_limit = [self.limits[2] - 1., self.limits[3] - 1.]

		while i < self.n_agents:
			p = np.random.uniform(low_limit, high_limit, (2,)).round(3)
			valid = True

			if not self.is_point_valid(p):
				continue

			for sample in valid_samples:
				if np.linalg.norm(p - sample) <= 2.5 * self.resolution:
					valid = False
					break

			if valid:
				valid_samples.append(p)
				i += 1

		return valid_samples

	def check_emptiness(self, agents):
		total_emptiness = True

		for _, agent in agents.items():
			if agent.alive and len(agent.frontier) > 0:
				total_emptiness = False
				break

		return total_emptiness

	def geodesic_partition(self, agents):
		metric_partition = dict([(i, [tuple(agents[i].gp)]) for i in range(self.n_agents)])
		metric_assignment = dict()

		# geod_part_start = time.time()
		while not self.check_emptiness(agents):
			deletion_updates = set()

			for k, agent in agents.items():
				if not agent.alive:
					continue

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

							distance = np.linalg.norm(np.array(agents[metric_assignment[expansion]].gp) - np.array(agent.gp))
							if (globals()["experiment_configuration"]["comm_range_test"] and 
								distance > globals()["experiment_configuration"]["comm_range"]):
								metric_assignment[expansion] = k
								continue

							deletion_updates.add((expansion, k))
							parent_of_deletion = agent.parents[expansion]
							normal = np.array(expansion) - np.array(parent_of_deletion)

							# print("k: {} - {}: ({}) - ma: {}".format(k, len(agent.borders), 
							# 	agent.borders.keys(), metric_assignment[expansion]))
							agent.borders[metric_assignment[expansion]].add(parent_of_deletion)
							agent.normals[parent_of_deletion] = normal

							agents[metric_assignment[expansion]].borders[k].add(expansion)
							agents[metric_assignment[expansion]].normals[expansion] = - normal

			for i in range(self.n_agents):
				f_i = agents[i].frontier

				for j in range(i + 1, self.n_agents):
					f_j = agents[j].frontier
					f_intersection = f_i & f_j

					for f_vmp in f_intersection:
						deletion_updates.add((f_vmp, i))
						deletion_updates.add((f_vmp, j))

			for d_upd in deletion_updates:
				try:
					agents[d_upd[1]].frontier.remove(d_upd[0])
				except Exception as e:
					pass

		for k, agent in agents.items():
			for obv in agent.orphan_border_vertices:
				if metric_assignment[obv] != k:
					continue

				expansion_neighbours = []
				for act in globals()["valid_actions"]:
					move = act[0] * agent.step_size
					nb_expand = np.array(obv) + move

					if not self._contracted_bnd.contains(shgeom.Point(nb_expand)):
						continue

					assigned_bfs_id = metric_assignment.get(tuple(nb_expand))
					if assigned_bfs_id is None or assigned_bfs_id == k:
						continue

					expansion_neighbours.append(assigned_bfs_id)

				if len(expansion_neighbours) > 0:
					agent.borders[min(expansion_neighbours)].add(obv)

		# geod_part_elapsed = time.time() - geod_part_start
		# print("Geodesic partitioning took: {}".format(geod_part_elapsed))

		for vmp, aid in metric_assignment.items():
			metric_partition[aid].append(vmp)

		return metric_partition

	@staticmethod
	def compute_work_continuous(region):
		if region is None:
			return 0.

		total_area = region.outer_boundary().area()
		for hole in region.holes:
			total_area -= hole.area()

		return total_area

	@staticmethod
	def compute_center_of_mass(poly):
		cm = np.zeros(2)
		if poly is None:
			return cm

		total_area = float(poly.area())
		for i in range(len(poly)):
			j = (i + 1) % len(poly)

			p1 = poly.coords[i]
			p2 = poly.coords[j]

			cm[0] += (p1[0] + p2[0]) * (p1[0] * p2[1] - p2[0] * p1[1])
			cm[1] += (p1[1] + p2[1]) * (p1[0] * p2[1] - p2[0] * p1[1])

		return (cm / (6. * total_area), total_area)

	@staticmethod
	def compute_geometric_center(region):
		total_area = 0.
		cm = np.zeros(2)
		if region is None:
			return cm

		ob_cm, ob_area = Environment.compute_center_of_mass(region.outer_boundary())
		cm += ob_cm * ob_area
		total_area += ob_area

		for hole in region.holes:
			hole_cm, hole_area = Environment.compute_center_of_mass(hole)
			cm += hole_cm * hole_area
			total_area += hole_area

		return cm / total_area


# ==============================================================================================
# ==============================================================================================
# ==============================================================================================


def runner_CD(period, env, agents, states, partitions, subregions):
	global experiment_configuration

	print("[CD] Experiment thread starts now.")

	experiment_start = time.time()
	iteration = 0

	for aid, agent in agents.items():
		experiment_configuration["agent_{}".format(aid)] = {
			"position": [],
			"workload": [],
			# "actual_work": [],
			"max_geod": []
		}

	while (not globals()["experiment_stop"] and 
		   iteration < experiment_configuration["global_iteration_threshold"]):

		cycle_start = time.time()
		iteration += 1

		if (experiment_configuration["random_kill"] and 
			iteration % experiment_configuration["random_kill_per"] == 0):

			rand_id = np.random.randint(experiment_configuration["agent_count"])
			agents[rand_id].alive = False
			states[rand_id]["alive"] = False

			experiment_configuration["random_kill_per"] += 2

		# ----------------------------------------- START ----------------------------------------
		for aid, agent in agents.items():
			if agent.alive:
				agent.reset()
				agent.borders = dict([(k, set([])) for k in range(globals()["agent_count"]) if k != aid])

		cycle_partitions = env.geodesic_partition(agents)

		for aid, agent in agents.items():
			if agent.alive:
				agent.work = len(cycle_partitions[aid])

		for aid, agent in agents.items():
			if not agent.alive:
				partitions[aid]["xcoords"] = []
				partitions[aid]["ycoords"] = []
				continue

			partition_aid_xs, partition_aid_ys = zip(*(cycle_partitions[aid]))
			partitions[aid]["xcoords"] = list(partition_aid_xs)
			partitions[aid]["ycoords"] = list(partition_aid_ys)

			agent.update(cycle_partitions[aid])

			states[aid]["pos"] = agent.gp

			experiment_configuration["agent_{}".format(aid)]["position"].append(tuple(agent.gp))
			experiment_configuration["agent_{}".format(aid)]["workload"].append(len(cycle_partitions[aid]))
			# experiment_configuration["agent_{}".format(aid)]["actual_work"].append(agent.actual_work)
			experiment_configuration["agent_{}".format(aid)]["max_geod"].append(agent.max_geod_dist)
		# ---------------------------------------- FINISH ----------------------------------------

		cycle_elapsed = time.time() - cycle_start
		print("[{}]- Elapsed: {}".format(iteration, cycle_elapsed))

	experiment_configuration["TIME"] = time.time() - experiment_start

	dump_experiment_log()

	print("[CD] xperiment thread ends now.")


def runner_GA(period, env, agents, states, partitions, subregions):
	global experiment_configuration

	print("[GA] Experiment thread starts now.")

	experiment_start = time.time()
	iteration = 0

	for aid, agent in agents.items():
		experiment_configuration["agent_{}".format(aid)] = {
			"position": [],
			"workload": [],
			# "actual_work": [],
			"max_geod": []
		}

	while (not globals()["experiment_stop"] and 
		   iteration < experiment_configuration["global_iteration_threshold"]):

		cycle_start = time.time()
		iteration += 1

		if (experiment_configuration["random_kill"] and 
			iteration % experiment_configuration["random_kill_per"] == 0):

			rand_id = np.random.randint(experiment_configuration["agent_count"])
			agents[rand_id].alive = False
			states[rand_id]["alive"] = False

			experiment_configuration["random_kill_per"] += 2

		# ----------------------------------------- START ----------------------------------------
		for aid, agent in agents.items():
			if agent.alive:
				agent.reset()

		# globals()["debug_intersections"].clear()

		work_regions = dict()

		for aid, agent in agents.items():
			if agent.alive:
				work_regions[aid] = agent.voronoi_subregion()
				agent.work = float(Environment.compute_work_continuous(work_regions[aid]))

			else:
				pass

		for aid, agent in agents.items():
			if not agent.alive:
				subregions[aid]["poly"] = None
				subregions[aid]["holes"] = []
				continue

			# agent.work_geometry = work_regions[aid]
			if work_regions[aid] is None:
				continue

			for ph in work_regions[aid].holes:
				if ph.orientation() == skgeom.Sign.POSITIVE:
					ph.reverse_orientation()

			for arh in globals()["environment"].actual_region.holes:
				if arh.orientation() == skgeom.Sign.POSITIVE:
					arh.reverse_orientation()

			work_cell = None

			try:
				intr_pieces = boolean_set.intersect(globals()["environment"].actual_region, 
													work_regions[aid])

				if len(intr_pieces) == 0:
					print("Agent {} has no intersection pieces!".format(aid))

				elif len(intr_pieces) > 1:
					# nb_disconnected = []
					# for i in range(len(intr_pieces)):
					# 	pass

					# print("Agent {} has disconnected piece(s) ({}) who neighbours: {}".format(
					# 	self.id, len(intr_pieces), nb_disconnected))

					for i in range(len(intr_pieces)):
						if inside_check(intr_pieces[i], agent.p):
							work_cell = intr_pieces[i]
							break

				else:
					work_cell = intr_pieces[0]

			except Exception as e:
				print("Agent {} - {}".format(aid, traceback.format_exc()))
				print("{}".format(agent.cvx_voronoi_cell))
				print("{}".format(agent.cvx_voronoi_cell.is_simple()))
				raise e

			if work_cell is None:
				continue

			if not isinstance(work_cell, skgeom.PolygonWithHoles):
				work_cell = skgeom.PolygonWithHoles(work_cell, [])

			# subregions[aid]["poly"] = work_regions[aid].outer_boundary()
			# subregions[aid]["holes"] = list(work_regions[aid].holes)
			subregions[aid]["poly"] = work_cell.outer_boundary()
			subregions[aid]["holes"] = list(work_cell.holes)

		for aid, agent in agents.items():
			if not agent.alive:
				continue

			# agent.update(work_regions[aid])
			agent.update(skgeom.PolygonWithHoles(subregions[aid]["poly"], subregions[aid]["holes"]))

			states[aid]["pos"] = agent.p

			experiment_configuration["agent_{}".format(aid)]["position"].append(tuple(agent.p))
			experiment_configuration["agent_{}".format(aid)]["workload"].append(agent.work)
			# experiment_configuration["agent_{}".format(aid)]["actual_work"].append(agent.actual_work)
			experiment_configuration["agent_{}".format(aid)]["max_geod"].append(agent.max_geod_dist)
		# ---------------------------------------- FINISH ----------------------------------------

		cycle_elapsed = time.time() - cycle_start
		print("[{}]- Elapsed: {}".format(iteration, cycle_elapsed))

	experiment_configuration["TIME"] = time.time() - experiment_start

	dump_experiment_log()

	print("[GA] Experiment thread ends now.")


def runner_G(period, env, agents, states, partitions, subregions):
	global experiment_configuration

	print("[G] Experiment thread starts now.")

	experiment_start = time.time()
	iteration = 0

	for aid, agent in agents.items():
		experiment_configuration["agent_{}".format(aid)] = {
			"position": [],
			"workload": [],
			# "actual_work": [],
			"max_geod": []
		}

	while (not globals()["experiment_stop"] and 
		   iteration < experiment_configuration["global_iteration_threshold"]):

		cycle_start = time.time()
		iteration += 1

		if (experiment_configuration["random_kill"] and 
			iteration % experiment_configuration["random_kill_per"] == 0):

			rand_id = np.random.randint(experiment_configuration["agent_count"])
			agents[rand_id].alive = False
			states[rand_id]["alive"] = False

			experiment_configuration["random_kill_per"] += 2

		# ----------------------------------------- START ----------------------------------------
		for aid, agent in agents.items():
			if agent.alive:
				agent.reset()

		# globals()["debug_intersections"].clear()

		work_regions = dict()

		for aid, agent in agents.items():
			if agent.alive:
				work_regions[aid] = agent.voronoi_subregion()
				agent.work = float(Environment.compute_work_continuous(work_regions[aid]))

			else:
				pass

		for aid, agent in agents.items():
			if not agent.alive:
				subregions[aid]["poly"] = None
				subregions[aid]["holes"] = []
				continue

			# agent.work_geometry = work_regions[aid]
			if work_regions[aid] is None:
				continue

			subregions[aid]["poly"] = work_regions[aid].outer_boundary()
			subregions[aid]["holes"] = list(work_regions[aid].holes)

			agent.update(work_regions[aid])

			states[aid]["pos"] = agent.p

			experiment_configuration["agent_{}".format(aid)]["position"].append(tuple(agent.p))
			experiment_configuration["agent_{}".format(aid)]["workload"].append(agent.work)
			# experiment_configuration["agent_{}".format(aid)]["actual_work"].append(agent.actual_work)
			experiment_configuration["agent_{}".format(aid)]["max_geod"].append(agent.max_geod_dist)
		# ---------------------------------------- FINISH ----------------------------------------

		cycle_elapsed = time.time() - cycle_start
		print("[{}]- Elapsed: {}".format(iteration, cycle_elapsed))

		# if cycle_elapsed < experiment_configuration["period"]:
		# 	time.sleep(experiment_configuration["period"] - cycle_elapsed)

	experiment_configuration["TIME"] = time.time() - experiment_start

	dump_experiment_log()

	print("[G] Experiment thread ends now.")


# ==============================================================================================
# ==============================================================================================
# ==============================================================================================


def animate_experiment(i, ax, lims, states, partitions, subregions):
	globals()["final_view_lock"].acquire()
	globals()["final_view_drawn"] = False

	ax.cla()
	ax.set_aspect("equal")
	ax.set_axis_off()
	ax.set_xlim(lims[0] - 5., lims[2] + 5.)
	ax.set_ylim(lims[1] - 5., lims[3] + 5.)

	if globals()["region_patch"] is not None:
		ax.add_patch(globals()["region_patch"])

	for _, obs_patch in globals()["obstacle_patches"].items():
		ax.add_patch(obs_patch)

	for aid, state in states.items():
		pos = state['pos']
		# pos, vel, hdg = state['pos'], state['vel'], state['hdg']

		if not state['alive']:
			continue

		robot_color = globals()['__COLORS'][aid]
		ax.add_artist(plt.Circle(tuple(pos), 1.5, color=robot_color))
		# ax.add_artist(plt.Circle(tuple(globals()["all_agents"][aid].goal), 1.5, color=robot_color))

		goal = globals()["all_agents"][aid].goal
		# ax.plot([goal[0]], [goal[1]], color=robot_color, marker="x", markersize="4")

		# if aid == 0 and len(globals()["debug_intersections"]) > 0:
		# 	a0_dip = np.array(globals()["debug_intersections"])
		# 	ax.scatter(a0_dip[:, 0], a0_dip[:, 1], s=4., color=robot_color, marker='x')

		m_hist = globals()["motion_history"].get(aid)
		if m_hist is not None:
			x_hist, y_hist = zip(*m_hist)
			ax.plot(x_hist, y_hist, color=robot_color)

		if globals()["experiment_configuration"]["distributed_algorithm"] == "continuous_dijkstra":
			gpart = partitions.get(aid)
			if gpart is not None:
				if len(gpart["xcoords"]) > 0 and len(gpart["xcoords"]) == len(gpart["ycoords"]):
					ax.scatter(gpart["xcoords"], gpart["ycoords"], s=8., color=robot_color, alpha=0.3)

		else:
			subreg = subregions.get(aid)
			if subreg is not None:
				if subreg.get("poly") is not None:
					ax.add_patch(plt.Polygon(subreg["poly"].coords, fill=True, color=robot_color, alpha=0.3))

					# if globals()["experiment_configuration"]["distributed_algorithm"] == "geodesic_approximate":
					# 	pwh = skgeom.PolygonWithHoles(subreg["poly"], subreg["holes"])
					# 	skel = skgeom.skeleton.create_interior_straight_skeleton(pwh)

					# 	for h in skel.halfedges:
					# 		if h.is_bisector:
					# 			p1 = h.vertex.point
					# 			p2 = h.opposite.vertex.point
					# 			ax.plot([p1.x(), p2.x()], [p1.y(), p2.y()], color=robot_color, lw=1)

	globals()["final_view_drawn"] = True
	globals()["final_view_lock"].release()


def get_total_path_traversed(pos_history):
	total = 0.

	for i in range(len(pos_history) - 1):
		total += np.linalg.norm(np.array(pos_history[i + 1]) - np.array(pos_history[i]))

	return total


def calculate_assignment_percentage(env, agents):
	assigned_area = 0.
	total_area = 0.

	total_area += float(env.actual_region.outer_boundary().area())
	for hole in env.actual_region.holes:
		total_area -= float(hole.area())

	for hole in env.actual_region.holes:
		if hole.orientation() == skgeom.Sign.POSITIVE:
			hole.reverse_orientation()

	for aid, agent in agents.items():
		work_cell = None

		try:
			intr_pieces = boolean_set.intersect(env.actual_region, agent.work_geometry)

			if len(intr_pieces) == 0:
				print("Agent {} has no intersection pieces!".format(aid))

			elif len(intr_pieces) > 1:
				for i in range(len(intr_pieces)):
					if inside_check(intr_pieces[i], agent.p):
						work_cell = intr_pieces[i]
						break

			else:
				work_cell = intr_pieces[0]

		except Exception as e:
			print("Agent {} - {}".format(aid, traceback.format_exc()))
			# print("{}".format(agent.cvx_voronoi_cell))
			# print("{}".format(agent.cvx_voronoi_cell.is_simple()))
			raise e

		if work_cell is not None:
			assigned_area += float(Environment.compute_work_continuous(work_cell))

	return assigned_area / total_area


def draw_unassigned_vs_assigned(env, agents, result_path=None, suffix=None):
	final_fig = plt.figure(num=2)
	f_ax = final_fig.add_subplot(1, 1, 1)

	lims = globals()["limits"]
	f_ax.cla()
	f_ax.set_aspect("equal")
	f_ax.set_axis_off()
	f_ax.set_xlim(lims[0] - 5., lims[2] + 5.)
	f_ax.set_ylim(lims[1] - 5., lims[3] + 5.)

	if globals()["region_patch"] is not None:
		# f_ax.add_patch(globals()["region_patch"])
		f_ax.add_patch(plt.Polygon(list(globals()["exp_region"]), fill=False, color=(0., 0., 0.)))

	# for _, obs_patch in globals()["obstacle_patches"].items():
	# 	f_ax.add_patch(obs_patch)

	for _, obs in globals()["exp_obstacles"].items():
		if len(obs) < 3:
			continue

		f_ax.add_patch(plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8))

	for aid, agent in agents.items():
		work_cell = None

		try:
			intr_pieces = boolean_set.intersect(env.actual_region, agent.work_geometry)

			if len(intr_pieces) == 0:
				print("Agent {} has no intersection pieces!".format(aid))

			elif len(intr_pieces) > 1:
				for i in range(len(intr_pieces)):
					if inside_check(intr_pieces[i], agent.p):
						work_cell = intr_pieces[i]
						break

			else:
				work_cell = intr_pieces[0]

		except Exception as e:
			print("Agent {} - {}".format(aid, traceback.format_exc()))
			print("{}".format(agent.cvx_voronoi_cell))
			print("{}".format(agent.cvx_voronoi_cell.is_simple()))
			raise e

		if work_cell is not None:
			robot_color = globals()["__COLORS"][aid]
			f_ax.add_artist(plt.Circle(tuple(agent.p), 1.5, color=robot_color))
			f_ax.add_patch(plt.Polygon(work_cell.outer_boundary().coords, fill=True, color=robot_color, alpha=0.3))

	fig_path = ""
	if result_path is not None:
		fig_path = "{}/final_assignment_view".format(result_path)

	if suffix is not None:
		fig_path += "_{}".format(suffix)

	if len(fig_path) > 0:
		plt.savefig("{}.png".format(fig_path))


def calculate_final_workloads(env, agents, result_path=None, suffix=None):
	# final_fig = plt.figure(num=2)
	# f_ax = final_fig.add_subplot(1, 1, 1)

	final_workloads = dict()
	for aid, agent in agents.items():
		if agent.alive:
			agent.reset()
			agent.borders = dict([(k, set([])) for k in range(globals()["agent_count"]) if k != aid])

	cycle_partitions = env.geodesic_partition(agents)
	total_work = 0

	for aid, agent in agents.items():
		partition_aid_xs, partition_aid_ys = zip(*(cycle_partitions[aid]))

		# robot_color = globals()["__COLORS"][aid]
		# if len(partition_aid_xs) > 0 and len(partition_aid_xs) == len(partition_aid_ys):
		# 	f_ax.scatter(partition_aid_xs, partition_aid_ys, s=8., color=robot_color, alpha=0.5)

		index = 0
		locations = dict()
		index_map = dict()

		for p in cycle_partitions[aid]:
			locations[p] = index
			index_map[index] = p
			index += 1

		N = len(locations)
		graph = np.zeros((N, N))

		for gp, gp_id in locations.items():
			v_gp = np.array(gp)

			for gp_np in agent.edges[gp]:
				v_np = np.array(gp_np)

				gp_np_id = locations.get(gp_np)
				if gp_np_id is not None:
					graph[gp_id, gp_np_id] = np.linalg.norm(v_gp - v_np)
					graph[gp_np_id, gp_id] = np.linalg.norm(v_gp - v_np)

		dists, preds = floyd_warshall(csgraph=graph, directed=False, return_predecessors=True)
		dists[dists == np.inf] = 0
		total_shortest_paths = np.sum(dists, axis=0)
		final_workloads[aid] = total_shortest_paths[locations[tuple(agent.gp)]]
		total_work += final_workloads[aid]

		print("Agent {} final workload: {}".format(aid, final_workloads[aid]))

	# fig_path = ""
	# if result_path is not None:
	# 	fig_path = "{}/final_view".format(result_path)

	# if suffix is not None:
	# 	fig_path += str(suffix)

	# if len(fig_path) > 0:
	# 	plt.savefig("{}.png".format(fig_path))

	for _, fw in final_workloads.items():
		fw /= total_work

	return final_workloads

def dump_experiment_log():
	global experiment_configuration

	experiment_configuration["global_agent_init_seed"] = globals()["global_agent_init_seed"]
	experiment_configuration["global_workload"] = globals()["environment"].global_workload

	vis_suffix = rospy.get_param("/behaviours/visibility", False)

	exp_results_dir = os.environ["HOME"] + "/thesis_ws/results"
	exp_meta_dir = "{}/{}{}/{}".format(experiment_configuration["coverage_name"], 
									   experiment_configuration["distributed_algorithm"], 
									   "VIS" if vis_suffix else "", 
									   experiment_configuration["agent_count"])

	full_dir_path = exp_results_dir + "/" + exp_meta_dir

	if not os.path.exists(full_dir_path):
		os.makedirs(full_dir_path)

	stamp = datetime.datetime.now()
	filename = "exp_history_{}.json".format(stamp)
	figname = "exp_history_{}.png".format(stamp)
	full_path = "{}/{}".format(full_dir_path, filename)

	i = 1
	while os.path.exists(full_path):
		rospy.logwarn("The file {} already exists!".format(filename))
		filename = "exp_history_{}_{}.json".format(stamp, i)
		figname = "exp_history_{}_{}.png".format(stamp, i)
		rospy.logwarn("Trying {}...".format(filename))

		full_path = "{}/{}".format(full_dir_path, filename)
		i += 1
		time.sleep(1.)

	data_shape = (experiment_configuration["agent_count"], 
				  experiment_configuration["global_iteration_threshold"])

	workloads = np.zeros(data_shape)
	# actual_works = np.zeros(data_shape)
	max_geods = np.zeros(data_shape)

	for aid in range(experiment_configuration["agent_count"]):
		workloads[aid] = experiment_configuration["agent_{}".format(aid)]["workload"]
		# actual_works[aid] = experiment_configuration["agent_{}".format(aid)]["actual_work"]
		max_geods[aid] = experiment_configuration["agent_{}".format(aid)]["max_geod"]

	experiment_configuration["results"] = dict()
	experiment_configuration["overall"] = calculate_final_workloads(globals()["environment"], globals()["all_agents"], full_dir_path, stamp)

	if experiment_configuration["distributed_algorithm"] != "continuous_dijkstra":
		experiment_configuration["percentage"] = calculate_assignment_percentage(globals()["environment"], globals()["all_agents"])

		if experiment_configuration["percentage"] > 1.:
			print("/*/*/*/*/*/*/")
			print("\tOver-assignment is detected: {} !".format(experiment_configuration["percentage"]))
			print("/*/*/*/*/*/*/")

	print("Final results:")

	print("=============================================")
	print("* Overall Std. Dev.    : {}".format(np.std(list(experiment_configuration["overall"].values()))))

	if experiment_configuration["distributed_algorithm"] == "continuous_dijkstra":
		print("* Assignment Percentage: 1.0")

	else:
		print("* Assignment Percentage: {}".format(experiment_configuration["percentage"]))

	print("=============================================")

	# print("* Workload metric      : {}".format())
	print("* Global workload      : {}".format(experiment_configuration["global_workload"]))
	print("* Workload sum.        : {}".format(np.sum(workloads[:, -1])))

	print("* Workload std. dev.   : {}".format(np.std(workloads[:, -1])))
	experiment_configuration["results"]["final_workload_std_dev"] = np.std(workloads[:, -1])

	print("* Workload mean        : {}".format(np.mean(workloads[:, -1])))
	experiment_configuration["results"]["final_workload_mean"] = np.mean(workloads[:, -1])

	# print("* Actual std. dev.     : {}".format(np.std(actual_works[:, -1])))
	# experiment_configuration["results"]["final_actual_std_dev"] = np.std(actual_works[:, -1])

	# print("* Actual mean          : {}".format(np.mean(actual_works[:, -1])))
	# experiment_configuration["results"]["final_actual_mean"] = np.mean(actual_works[:, -1])

	print("* Max geod. std. dev.  : {}".format(np.std(max_geods[:, -1])))
	experiment_configuration["results"]["final_max_geod_std_dev"] = np.std(max_geods[:, -1])

	print("* Max geod mean        : {}".format(np.mean(max_geods[:, -1])))
	experiment_configuration["results"]["final_max_geod_mean"] = np.mean(max_geods[:, -1])

	# traversals = np.zeros(experiment_configuration["agent_count"])
	# print("* Path length traversed:")
	# for aid in range(experiment_configuration["agent_count"]):
	# 	length = get_total_path_traversed(experiment_configuration["agent_{}".format(aid)]["position"])
	# 	print("\t** Agent {}: {}".format(aid, length))
	# 	traversals[aid] = length

	# print("* Path length mean     : {}".format(np.mean(traversals)))
	# print("* Path length std. dev.: {}".format(np.std(traversals)))
	# experiment_configuration["results"]["traversals_mean"] = np.mean(traversals)
	# experiment_configuration["results"]["traversals_std_dev"] = np.std(traversals)

	# Dump with results
	with open(full_path, "w") as H:
		json.dump(experiment_configuration, H, indent=4)

	print("Dumped experimental history to {}".format(full_path))

	globals()["final_view_lock"].acquire()
	if globals()["final_view_drawn"]:
		plt.savefig("{}/{}".format(full_dir_path, figname))
		# plt.savefig("{}/{}".format(full_dir_path, figname), bbox_inches="tight")
	globals()["final_view_lock"].release()

	if experiment_configuration["distributed_algorithm"] != "continuous_dijkstra":
		globals()["final_view_lock"].acquire()
		draw_unassigned_vs_assigned(globals()["environment"], globals()["all_agents"], full_dir_path, stamp)
		globals()["final_view_lock"].release()


def custom_sigint_handler(signal, frame):
	globals()["experiment_stop"] = True

	if globals()["experiment_thread"] is not None:
		globals()["experiment_thread"].join()

	print("SIGINT!!")
	sys.exit(1)

def custom_sigterm_handler(signal, frame):
	globals()["experiment_stop"] = True

	if globals()["experiment_thread"] is not None:
		globals()["experiment_thread"].join()

	print("SIGTERM!!")
	sys.exit(2)


if __name__ == "__main__":
	agent_count = int(sys.argv[1])

	if agent_count > len(__COLORS):
		additional_colors = [tuple(np.random.rand(3)) for j in range(agent_count - len(__COLORS) + 1)]
		__COLORS.extend(additional_colors)

	rospy.init_node("synchronized_coverage", anonymous=False, disable_signals=True)

	signal.signal(signal.SIGINT, custom_sigint_handler)
	signal.signal(signal.SIGTERM, custom_sigterm_handler)

	# ---------------------------------------------------------------------------------------------
	# Load parameters
	experiment_configuration["agent_count"] = agent_count
	experiment_configuration["period"] = rospy.get_param("/motion_params/delta_t", 1.)
	experiment_configuration["coverage_name"] = rospy.get_param("/coverage_name", "NONE")
	experiment_configuration["distributed_algorithm"] = rospy.get_param("/centroid_alg", "NONE")

	experiment_configuration["K_rep"] = rospy.get_param("/motion_params/K_repulsion", 1.)
	experiment_configuration["K_att"] = rospy.get_param("/motion_params/K_attraction", 1.)
	experiment_configuration["phys_radius"] = rospy.get_param("/motion_params/physical_radius", 0.5)
	experiment_configuration["sense_radius_scale"] = rospy.get_param("/sensing_params/sense_fp_phys_rad_scale", 4)
	experiment_configuration["resolution"] = experiment_configuration["phys_radius"] * experiment_configuration["sense_radius_scale"]

	if experiment_configuration["distributed_algorithm"] != "continuous_dijkstra":
		experiment_configuration["global_iteration_threshold"] = rospy.get_param("/continuous_iteration_threshold", 10)
		experiment_configuration["convergence_movement_threshold"] = experiment_configuration["phys_radius"] * 2.

	else:
		experiment_configuration["global_iteration_threshold"] = rospy.get_param("/discrete_iteration_threshold", 10)
		experiment_configuration["convergence_movement_threshold"] = experiment_configuration["resolution"]

	experiment_configuration["random_kill"] = rospy.get_param("/random_kill", False)
	experiment_configuration["random_kill_per"] = rospy.get_param("/random_kill_per", 10)
	experiment_configuration["comm_range"] = rospy.get_param("/comm_range", 100)
	experiment_configuration["comm_range_test"] = rospy.get_param("/comm_range_test", False)

	limits = None
	exp_region = rospy.get_param("/coverage_boundary", [])
	experiment_configuration["coverage_region"] = exp_region
	if len(exp_region) < 3:
		rospy.logerr("Region of interest has less than 3 vertices!")
		sys.exit(1)

	else:
		xcoords, ycoords = zip(*exp_region)
		xmin, xmax = min(xcoords), max(xcoords)
		ymin, ymax = min(ycoords), max(ycoords)
		limits = (xmin, ymin, xmax, ymax)
		region_patch = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))

	exp_obstacles = rospy.get_param("/coverage_obstacles", dict())
	experiment_configuration["coverage_obstacles"] = exp_obstacles
	for obs_name, obs in exp_obstacles.items():
		if len(obs) < 3:
			# rospy.logerr("Invalid obstacle polygon!")
			# sys.exit(1)
			continue

		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)

	agent_init_rand_seed_key = rospy.search_param("agent_init_rand_seed")
	if agent_init_rand_seed_key is not None:
		global_agent_init_seed = rospy.get_param(agent_init_rand_seed_key)
		experiment_configuration["global_agent_init_seed"] = global_agent_init_seed
		np.random.seed(global_agent_init_seed)

	else:
		rospy.logwarn("No agent initial location random seed is found! Reverting to random seeding.")
		experiment_configuration["global_agent_init_seed"] = np.random.rand() * 1e5
		np.random.seed(experiment_configuration["global_agent_init_seed"])

	print("Loaded experiment parameters.")
	print("Sensing scale: {}".format(experiment_configuration["sense_radius_scale"]))
	print("Phys. radius: {}".format(experiment_configuration["phys_radius"]))
	print("Resolution: {}".format(experiment_configuration["resolution"]))

	# ---------------------------------------------------------------------------------------------
	# Initialize
	environment = Environment(experiment_configuration["coverage_region"], 
							  experiment_configuration["coverage_obstacles"], 
							  experiment_configuration["resolution"])

	print("Initialized coverage environment.")

	rand_positions = environment.generate_random_positions(agent_count)
	experiment_configuration["agent_initial_positions"] = [tuple(p) for p in rand_positions]

	exp_mode = "CD"
	if experiment_configuration["distributed_algorithm"] == "continuous_dijkstra":
		exp_mode = "CD"

	elif experiment_configuration["distributed_algorithm"] == "geodesic_approximate":
		exp_mode = "GA"

	elif experiment_configuration["distributed_algorithm"] == "geometric":
		exp_mode = "G"

	else:
		print("Unknown mode! Exiting...")
		sys.exit(1)

	R = experiment_configuration["resolution"]
	for i in range(agent_count):
		p = rand_positions[i]
		theta = np.random.uniform(-np.pi, np.pi)
		gpos = np.array(np.round(p / R) * R)

		all_agents[i] = BFSAgent(i, rand_positions[i], gpos, R, exp_mode)
		all_agents[i].borders = dict([(k, set([])) for k in range(agent_count) if k != i])

		all_partitions[i] = {
			"xcoords": [],
			"ycoords": []
		}

		all_subregions[i] = {
			"poly": None,
			"holes": []
		}

		all_states[i] = {
			"pos": p,
			"alive": True,
			"seq": 0,
			"workload": 0.
		}

	print("Initialized coverage agents ({} states, {} partitions, {} agents).".format(
		len(all_states), len(all_partitions), len(all_agents)))

	# ---------------------------------------------------------------------------------------------
	# Start experiment / partitioning
	target_func = None

	if experiment_configuration["distributed_algorithm"] == "continuous_dijkstra":
		target_func = runner_CD

	elif experiment_configuration["distributed_algorithm"] == "geodesic_approximate":
		target_func = runner_GA

	elif experiment_configuration["distributed_algorithm"] == "geometric":
		target_func = runner_G

	else:
		print("Unknown algorithm! Exiting...")
		sys.exit(1)

	experiment_thread = threading.Thread(name="Supervisor", 
										 target=target_func, 
										 args=(experiment_configuration["period"], 
										 	   environment, 
											   all_agents, 
											   all_states, 
											   all_partitions, 
											   all_subregions))
	experiment_thread.start()

	# ---------------------------------------------------------------------------------------------
	# Visualize
	main_figure = plt.figure()
	ax = main_figure.add_subplot(1, 1, 1)
	ani = animation.FuncAnimation(main_figure, 
								  partial(animate_experiment, 
										  ax=ax,
										  lims=limits,
										  states=all_states,
										  partitions=all_partitions, 
										  subregions=all_subregions),
								  interval=200)

	plt.show()
