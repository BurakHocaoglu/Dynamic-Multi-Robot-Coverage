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
plt.rcParams['animation.ffmpeg_path'] = '/usr/local/bin/ffmpeg'

from functools import partial
from collections import deque

import shapely.geometry as sg

from std_srvs.srv import Trigger

from coverage_control2.msg import AgentState, Polygon, HistoryStep, PolygonWithHoles, GeodesicPartition
from coverage_control2.srv import SetInitialPose, SetId, PrintMass

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.5,0.5,0.5), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.99,0.99,0)]

all_states = dict()
all_vpolygons = dict()
all_cvx_voronoi = dict()
all_vl_voronoi = dict()
motion_history = dict()
all_vlv_history = dict()
all_geodesic_partitions = dict()
got_first = False

plotting_title = "Dist. Cov. Experiment"

exp_region = []
region_patch = None
exp_obstacles = dict()
obstacle_patches = dict()

visibility_focus_id = -1
visibility_focus_level = 2
visibility_focus_level = 4
global_agent_init_seed = 0

global_iteration_threshold = 10
coverage_name = ""
distributed_algorithm = ""
workload_scale = 1
global_workload = 0
global_sequence = 0
sequence_stop = False
seq_thread = None
seq_tolerance = 3

collect_states = True
convergence_conditions = dict()
convergence_movement_threshold = 1.

def get_area(polygon):
	area = 0
	for i in range(len(polygon) - 1):
		x_i, y_i = polygon[i]
		x_j, y_j = polygon[i + 1]
		area += x_i * y_j - x_j * y_i

	return area * 0.5

def is_point_valid(bnd, obs, p):
	if not bnd.contains(sg.Point(p)):
		return False

	for _, o in obs.items():
		if o.contains(sg.Point(p)):
			return False

	return True

def get_random_positions(bnd, obs, lims, count):
	valid_samples, i = [], 0
	low_limit = [lims[0] + 1., lims[1] + 1.]
	high_limit = [lims[2] - 1., lims[3] - 1.]
	D = rospy.get_param("/sensing_params/sense_fp_phys_rad_scale", 2)

	while i < count:
		p = np.random.uniform(low_limit, high_limit, (2,)).round(3)
		valid = True

		if not is_point_valid(bnd, obs, p):
			continue

		for sample in valid_samples:
			# if np.linalg.norm(p - sample) <= 10.:
			if np.linalg.norm(p - sample) <= 2.5 * D:
				valid = False
				break

		if valid:
			valid_samples.append(p)
			i += 1

	return valid_samples

def compute_global_workload(region, holes, space="c", resolution=1.):
	if space == 'c':
		negative_workload = 0.

		# for _, hole in holes.items():
		# 	negative_workload += hole.area()

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

	else:
		rospy.logwarn("Unknown space type {}!".format(space))
		return 0.

def check_convergence():
	result = True

	for _, stillness in globals()["convergence_conditions"].items():
		result = result & (stillness >= 20)

	return result

def state_cb(msg):
	# print("Monitor got from {} - {}".format(msg.id, msg.is_alive))
	globals()["all_states"][msg.id] = {"pos": np.array([msg.position.x, msg.position.y], dtype=float), 
									   "vel": np.array([msg.velocity.x, msg.velocity.y], dtype=float), 
									   "goal": np.array([msg.goal.x, msg.goal.y], dtype=float), 
									   "cntr": np.array([msg.centroid.x, msg.centroid.y], dtype=float), 
									   "hdg": msg.heading, 
									   "alive": msg.is_alive,
									   "seq": msg.seq,
									   "dom": msg.largest_workload_piece, 
									   "load": msg.workload}

	if globals()["motion_history"].get(msg.id) is None:
		# globals()["motion_history"][msg.id] = deque(maxlen=100)
		globals()["motion_history"][msg.id] = deque()

	curr = np.array([msg.position.x, msg.position.y], dtype=float)

	if len(globals()["motion_history"].get(msg.id)) > 1:
		prev = globals()["motion_history"][msg.id][-1]

		if np.linalg.norm(curr - prev) < globals()["convergence_movement_threshold"]:
			globals()["convergence_conditions"][msg.id] += 1

		else:
			globals()["convergence_conditions"][msg.id] = 0

	# if abs(msg.seq - globals()["global_sequence"]) < globals()["seq_tolerance"]:
	# 	globals()["motion_history"][msg.id].append(curr)
	# else:
	# 	print("Agent {} has fallen behind! Set alive to False!".format(msg.id))
	# 	globals()["all_states"][msg.id]['alive'] = False

def vpoly_cb(msg):
	projected_poly = [(v.x, v.y) for v in msg.points]
	globals()["all_vpolygons"][msg.id] = plt.Polygon(projected_poly, 
													 fill=True, 
													 color=globals()["__COLORS"][msg.id], 
													 alpha=0.15)

def voronoi_cb(msg):
	projected_poly = [(v.x, v.y) for v in msg.points]
	globals()["all_cvx_voronoi"][msg.id] = plt.Polygon(projected_poly, 
													   fill=True, 
													   color=globals()["__COLORS"][msg.id], 
													   alpha=0.15)

def vlv_poly_cb(msg):
	if not globals()["collect_states"]:
		rospy.logwarn("Everyone has passed the iteration threshold ({})!".format(globals()["global_iteration_threshold"]))
		return

	projected_poly = [(v.x, v.y) for v in msg.cell.outer_boundary.points]
	projected_holes = []

	for poly in msg.cell.holes:
		projected_holes.append([(v.x, v.y) for v in poly.points])

	if len(projected_poly) > 2:
		globals()["all_vl_voronoi"][msg.id] = plt.Polygon(projected_poly, 
														  fill=True, 
														  color=globals()["__COLORS"][msg.id], 
														  alpha=0.2)

		if globals()["all_vlv_history"].get(msg.id) is None:
			globals()["all_vlv_history"][msg.id] = dict()
			globals()["all_vlv_history"][msg.id]["position"] = []
			globals()["all_vlv_history"][msg.id]["polygon"] = []
			globals()["all_vlv_history"][msg.id]["holes"] = []
			globals()["all_vlv_history"][msg.id]["workloads"] = []

		globals()["all_vlv_history"][msg.id]["position"].append((msg.position.x, msg.position.y))
		globals()["all_vlv_history"][msg.id]["polygon"].append(projected_poly)
		globals()["all_vlv_history"][msg.id]["holes"].append(projected_holes)
		globals()["all_vlv_history"][msg.id]["workloads"].append(msg.workload)

	else:
		print("{} - Invalid poly".format(msg.id))

	history_lengths = [len(hist["position"]) for _, hist in globals()["all_vlv_history"].items()]

	if min(history_lengths) >= globals()["global_iteration_threshold"]:
		rospy.logwarn("Everyone has passed the iteration threshold ({})!".format(globals()["global_iteration_threshold"]))
		globals()["collect_states"] = False

	# if check_convergence():
	# 	globals()["collect_states"] = False

def geodesic_partition_cb(msg):
	if not globals()["collect_states"]:
		rospy.logwarn("Everyone has passed the iteration threshold ({})!".format(globals()["global_iteration_threshold"]))
		return

	# print("Agent {}".format(msg.id))

	if not globals()["got_first"]:
		globals()["got_first"] = True
		globals()["all_vlv_history"]["T_start"] = time.time()

	if globals()["all_geodesic_partitions"].get(msg.id) is None:
		globals()["all_geodesic_partitions"][msg.id] = {"xcoords": [], "ycoords": []}
		globals()["all_vlv_history"][msg.id] = {"position": [], "workloads": [], "max_geod_dist": []}

	globals()["all_geodesic_partitions"][msg.id]["xcoords"] = msg.xcoords
	globals()["all_geodesic_partitions"][msg.id]["ycoords"] = msg.ycoords
	globals()["all_vlv_history"][msg.id]["position"].append((msg.position.x, msg.position.y))
	globals()["all_vlv_history"][msg.id]["workloads"].append(msg.workload)

	p = np.array((msg.position.x, msg.position.y))
	partition = np.array((msg.xcoords, msg.ycoords)).T
	globals()["all_vlv_history"][msg.id]["max_geod_dist"].append(max(np.linalg.norm(partition - p, axis=1)))

	# print("{} - {}".format(type(globals()["all_vlv_history"]), type(globals()["all_vlv_history"].values()[0])))
	# history_lengths = [len(hist["position"]) for _, hist in globals()["all_vlv_history"].items()]
	history_lengths = []
	for _, hist in globals()["all_vlv_history"].items():
		if not isinstance(hist, dict):
			continue

		history_lengths.append(len(hist["position"]))

	if min(history_lengths) >= globals()["global_iteration_threshold"]:
		rospy.logwarn("Everyone has passed the iteration threshold ({})!".format(globals()["global_iteration_threshold"]))
		globals()["collect_states"] = False

	# if check_convergence():
	# 	globals()["collect_states"] = False

def handle_vp_focus(req):
	try:
		globals()["visibility_focus_id"] = req.id
		return True
	except Exception as e:
		print(traceback.format_exc())
		return False

def handle_vis_level(req):
	try:
		globals()["visibility_focus_level"] = req.id
		return True
	except Exception as e:
		print(traceback.format_exc())
		return False

def handle_print_mass(req):
	try:
		# message = ""
		message = []

		for vid, vlv in globals()["all_vl_voronoi"].items():
			# message += "({}, {}) - ".format(vid, get_area(vlv.get_xy()))
			# message += "({}, {}) - ".format(vid, globals()["all_states"][vid]["load"])
			message.append("({}, {}, {})".format(vid, globals()["all_states"][vid]["load"], 
													  globals()["all_states"][vid]["dom"]))

		return True, message
	except Exception as e:
		print(traceback.format_exc())
		return False, "FAIL"

def animate_experiment(i, ax, lims, S, VP, VLV):
	# ax.clear()
	ax.cla()
	ax.set_aspect("equal")
	# ax.set_title("Dist. Cov. Experiment")
	# ax.set_title(globals()["plotting_title"])
	ax.set_axis_off()
	ax.set_xlim(lims[0] - 5., lims[2] + 5.)
	ax.set_ylim(lims[1] - 5., lims[3] + 5.)

	if globals()["region_patch"] is not None:
		ax.add_patch(globals()["region_patch"])

	for _, obs_patch in globals()["obstacle_patches"].items():
		ax.add_patch(obs_patch)

	for aid, state in S.items():
		# State visualization
		pos, vel, hdg, goal, alive = state['pos'], state['vel'], state['hdg'], state['goal'], state['alive']
		# cntr = state['cntr']

		if not alive:
			# print("Animation: {} isn't alive. Skipping...".format(aid))
			continue
		# else:
		# 	print("Drawing {}...".format(aid))

		# if abs(globals()["global_sequence"] - state['seq']) >= globals()["seq_tolerance"]:
		# 	state['alive'] = False
		# 	continue
		# else:
		# 	print("Agent {}({}) - Seq: {}, Global: {}".format(
		# 		aid, alive, state['seq'], globals()["global_sequence"]))

		robot_color = globals()['__COLORS'][aid]
		# cntr_color = globals()['__COLORS'][aid - 1]
		# ax.quiver(pos[0], pos[1], np.cos(hdg), np.sin(hdg), color=robot_color)
		ax.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
		# ax.add_artist(plt.Circle(tuple(goal), 2., color=robot_color))
		# ax.add_artist(plt.Circle(tuple(cntr), 1., color=cntr_color))

		m_hist = globals()["motion_history"][aid]
		if len(m_hist) > 0:
			x_hist, y_hist = zip(*m_hist)
			ax.plot(x_hist, y_hist, color=robot_color)

		if globals()["visibility_focus_level"] == 1:
			cvxpoly = globals()["all_cvx_voronoi"].get(aid)
			if cvxpoly is not None:
				ax.add_patch(cvxpoly)

		elif globals()["visibility_focus_level"] == 2:
			vlvpoly = globals()["all_vl_voronoi"].get(aid)
			if vlvpoly is not None:
				ax.add_patch(vlvpoly)

		elif globals()["visibility_focus_level"] == 3:
			vpoly = globals()["all_vpolygons"].get(aid)
			if vpoly is not None:
				ax.add_patch(vpoly)

		elif globals()["visibility_focus_level"] == 4:
			gpartition = globals()["all_geodesic_partitions"].get(aid)
			if gpartition is not None:
				ax.scatter(gpartition["xcoords"], gpartition["ycoords"], s=8., 
							color=robot_color, alpha=0.5)

def dump_experiment_data():
	vis_suffix = rospy.get_param("/behaviours/visibility", False)

	exp_results_dir = os.environ["HOME"] + "/thesis_ws/results"
	exp_meta_dir = "{}/{}{}/{}".format(globals()["coverage_name"], 
									   globals()["distributed_algorithm"], 
									   "VIS" if vis_suffix else "", 
									   globals()["agent_count"])

	full_dir_path = exp_results_dir + "/" + exp_meta_dir

	if not os.path.exists(full_dir_path):
		os.makedirs(full_dir_path)

	stamp = datetime.datetime.now()
	filename = "exp_history_{}.json".format(stamp)
	full_path = "{}/{}".format(full_dir_path, filename)

	if os.path.exists(full_path):
		rospy.logwarn("The file {} already exists. Not dumping anything!".format(filename))

	else:
		globals()["all_vlv_history"]["global_agent_init_seed"] = globals()["global_agent_init_seed"]
		globals()["all_vlv_history"]["agent_count"] = globals()["agent_count"]
		globals()["all_vlv_history"]["global_workload"] = globals()["global_workload"]
		globals()["all_vlv_history"]["coverage_region"] = rospy.get_param("/coverage_boundary", [])
		globals()["all_vlv_history"]["coverage_obstacles"] = rospy.get_param("/coverage_obstacles", dict())
		globals()["all_vlv_history"]["agent_initial_positions"] = [tuple(p) for p in globals()["rand_positions"]]
		globals()["all_vlv_history"]["workload_scale"] = globals()["workload_scale"]
		globals()["all_vlv_history"]["env_name"] = rospy.get_param("/coverage_name", "")

		with open(full_path, "w") as H:
			json.dump(globals()["all_vlv_history"], H, indent=4)

		print("Dumped experimental history to {}".format(full_path))

def load_experiment_metadata():
	globals()["coverage_name"] = rospy.get_param("/coverage_name", "NONE")
	globals()["distributed_algorithm"] = rospy.get_param("/centroid_alg", "NONE")

	phys_radius = rospy.get_param("/physical_radius", 0.5)
	sense_radius_scale = rospy.get_param("/sense_fp_phys_rad_scale", 4)
	globals()["workload_scale"] = phys_radius * sense_radius_scale;

	if globals()["distributed_algorithm"] != "continuous_dijkstra":
		globals()["global_iteration_threshold"] = rospy.get_param("/continuous_iteration_threshold", 10)
		globals()["convergence_movement_threshold"] = phys_radius * 2.

	else:
		globals()["global_iteration_threshold"] = rospy.get_param("/discrete_iteration_threshold", 10)
		globals()["convergence_movement_threshold"] = globals()["workload_scale"]

def sequence_thread(period):
	print("Sequence thread is on.")

	while not globals()["sequence_stop"]:
		t_start = time.time()
		globals()["global_sequence"] += 1
		t_passed = time.time() - t_start

		if t_passed > period:
			printf("Sequence thread missed.")
		else:
			time.sleep(period - t_passed)

	print("Sequence thread is off.")

def customSigIntHandler(signum, frame):
	globals()["collect_states"] = False
	globals()["sequence_stop"] = True

	if globals()["seq_thread"] is not None:
		globals()["seq_thread"].join()

	globals()["all_vlv_history"]["T_end"] = time.time()
	# dump_experiment_data()

if __name__ == "__main__":
	agent_count = int(sys.argv[1])

	for i in range(agent_count):
		convergence_conditions[i + 1] = 0

	if agent_count > len(__COLORS):
		additional_colors = [tuple(np.random.rand(3)) for j in range(agent_count - len(__COLORS) + 1)]
		__COLORS.extend(additional_colors)

	rospy.init_node("monitor", anonymous=False, disable_signals=True)

	signal.signal(signal.SIGINT, customSigIntHandler)

	states_sub = rospy.Subscriber("/states", AgentState, state_cb, queue_size=20)
	vpoly_sub = rospy.Subscriber("/visibility_polys", Polygon, vpoly_cb, queue_size=20)
	cvx_vor_sub = rospy.Subscriber("/convex_voronoi", Polygon, voronoi_cb, queue_size=20)
	vlv_poly_sub = rospy.Subscriber("/visibility_limited_voronoi", HistoryStep, vlv_poly_cb, queue_size=20)
	geodesic_partition_sub = rospy.Subscriber("/geodesic_partition", GeodesicPartition, geodesic_partition_cb, queue_size=20)

	vp_vis_focus_service = rospy.Service("/set_vp_vis_focus", SetId, handle_vp_focus)
	vis_focus_level_service = rospy.Service("/set_vis_focus_level", SetId, handle_vis_level)
	print_instantaneous_mass = rospy.Service("/print_instantaneous_mass", PrintMass, handle_print_mass)

	plotting_title = rospy.get_param("/plotting_title", "Dist. Cov. Experiment")
	seq_tolerance = rospy.get_param("/behaviours/communication_tolerance", 3)

	load_experiment_metadata()

	limits = None
	exp_region = rospy.get_param("/coverage_boundary", [])
	if len(exp_region) < 3:
		rospy.logerr("Region of interest has less than 3 vertices!")
		sys.exit(1)

	else:
		xcoords, ycoords = zip(*exp_region)
		xmin, xmax = min(xcoords), max(xcoords)
		ymin, ymax = min(ycoords), max(ycoords)
		limits = (xmin, ymin, xmax, ymax)
		region_patch = plt.Polygon(list(exp_region), fill=False, color=(0., 0., 0.))
		exp_region = sg.Polygon(exp_region).buffer(-1.)

	exp_obstacles = rospy.get_param("/coverage_obstacles", dict())
	for obs_name, obs in exp_obstacles.items():
		if len(obs) < 3:
			rospy.logerr("Invalid obstacle polygon!")
			sys.exit(1)

		obstacle_patches[obs_name] = plt.Polygon(list(obs), fill=True, color=(0., 0., 0.), alpha=0.8)
		exp_obstacles[obs_name] = sg.Polygon(obs).buffer(1.)

	agent_init_rand_seed_key = rospy.search_param("agent_init_rand_seed")
	if agent_init_rand_seed_key is not None:
		global_agent_init_seed = rospy.get_param(agent_init_rand_seed_key)
		np.random.seed(global_agent_init_seed)

	else:
		rospy.logwarn("No agent initial location random seed is found! Reverting to random seeding.")

	global_workload = compute_global_workload(exp_region, exp_obstacles, 
		space="d" if distributed_algorithm == "continuous_dijkstra" else "c", 
		resolution=workload_scale)

	rand_positions = get_random_positions(exp_region, exp_obstacles, limits, agent_count)
	# rand_positions = [np.array([-70., 10.]), 
	# 				  np.array([30., -60.]), 
	# 				  np.array([-30., -20.]), 
	# 				  np.array([70., 70.])]

	for i in range(agent_count):
		p = rand_positions[i]
		theta = np.random.uniform(-np.pi, np.pi)

		init_pos_service_i = "/Agent{}/set_initial_pose".format(i + 1)
		rospy.wait_for_service(init_pos_service_i)

		init_pos_client = rospy.ServiceProxy(init_pos_service_i, SetInitialPose)
		res = init_pos_client(p[0], p[1], theta)

		if res:
			rospy.loginfo("Agent {} will start with ({}, {})".format(i + 1, p[0], p[1]))

		else:
			rospy.logerr("Failed to set Agent {} initial pose!".format(i + 1))
			sys.exit(1)

	for i in range(agent_count):
		ready_service_i = "/Agent{}/set_ready".format(i + 1)
		rospy.wait_for_service(ready_service_i)

		set_ready_client = rospy.ServiceProxy(ready_service_i, Trigger)
		res = set_ready_client()

		if res:
			rospy.loginfo("Agent {} ready is confirmed.".format(i + 1))

		else:
			rospy.logerr("Failed to set Agent {} ready!".format(i + 1))
			sys.exit(1)

	seq_thread = threading.Thread(name="Sequencer", 
								  target=sequence_thread, 
								  args=(rospy.get_param("/motion_params/delta_t", 1.), ))
	seq_thread.start()

	figure = plt.figure()
	exp_ax = figure.add_subplot(1, 1, 1)
	ani_func = animation.FuncAnimation(figure, 
									   partial(animate_experiment, 
									   		   ax=exp_ax, 
									   		   lims=limits, 
									   		   S=all_states, 
									   		   VP=all_vpolygons, 
									   		   VLV=all_vl_voronoi), 
									   interval=200, 
									   save_count=50)

	rospy.loginfo("Experiment will be visualized, now...")
	plt.show(block=True)

	rospy.spin()
