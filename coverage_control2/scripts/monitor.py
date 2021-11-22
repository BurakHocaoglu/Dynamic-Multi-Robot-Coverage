#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import json
import rospy
import signal
import traceback
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation

from functools import partial
from collections import deque

import shapely.geometry as sg

from std_srvs.srv import Trigger

from coverage_control2.msg import AgentState, Polygon, HistoryStep, PolygonWithHoles
from coverage_control2.srv import SetInitialPose, SetId, PrintMass

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

all_states = dict()
all_vpolygons = dict()
all_cvx_voronoi = dict()
all_vl_voronoi = dict()
motion_history = dict()
all_vlv_history = dict()

exp_region = []
region_patch = None
exp_obstacles = dict()
obstacle_patches = dict()

visibility_focus_id = -1
visibility_focus_level = 0
global_agent_init_seed = None

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

	while i < count:
		p = np.random.uniform(low_limit, high_limit, (2,)).round(3)
		valid = True

		if not is_point_valid(bnd, obs, p):
			continue

		for sample in valid_samples:
			if np.linalg.norm(p - sample) <= 5.:
				valid = False
				break

		if valid:
			valid_samples.append(p)
			i += 1

	return valid_samples

def state_cb(msg):
	globals()["all_states"][msg.id] = {"pos": np.array([msg.position.x, msg.position.y], dtype=float), 
									   "vel": np.array([msg.velocity.x, msg.velocity.y], dtype=float), 
									   "goal": np.array([msg.goal.x, msg.goal.y], dtype=float), 
									   "hdg": msg.heading, 
									   "dom": msg.largest_workload_piece, 
									   "load": msg.workload}

	if globals()["motion_history"].get(msg.id) is None:
		globals()["motion_history"][msg.id] = deque(maxlen=100)

	globals()["motion_history"][msg.id].append(np.array([msg.position.x, msg.position.y], dtype=float))

def vpoly_cb(msg):
	projected_poly = [(v.x, v.y) for v in msg.points]
	globals()["all_vpolygons"][msg.id] = plt.Polygon(projected_poly, 
													 fill=True, 
													 color=globals()["__COLORS"][msg.id], 
													 alpha=0.25)

def voronoi_cb(msg):
	projected_poly = [(v.x, v.y) for v in msg.points]
	globals()["all_cvx_voronoi"][msg.id] = plt.Polygon(projected_poly, 
													   fill=True, 
													   color=globals()["__COLORS"][msg.id], 
													   alpha=0.25)

def vlv_poly_cb(msg):
	projected_poly = [(v.x, v.y) for v in msg.cell.outer_boundary.points]
	projected_holes = []

	for poly in msg.cell.holes:
		projected_holes.append([(v.x, v.y) for v in poly.points])

	if len(projected_poly) > 2:
		globals()["all_vl_voronoi"][msg.id] = plt.Polygon(projected_poly, 
														  fill=True, 
														  color=globals()["__COLORS"][msg.id], 
														  alpha=0.3)

		if globals()["all_vlv_history"].get(msg.id) is None:
			globals()["all_vlv_history"][msg.id] = dict()
			globals()["all_vlv_history"][msg.id]["position"] = []
			globals()["all_vlv_history"][msg.id]["polygon"] = []
			globals()["all_vlv_history"][msg.id]["holes"] = []

		globals()["all_vlv_history"][msg.id]["position"].append((msg.position.x, msg.position.y))
		globals()["all_vlv_history"][msg.id]["polygon"].append(projected_poly)
		globals()["all_vlv_history"][msg.id]["holes"].append(projected_holes)

	else:
		print("{} - Invalid poly".format(msg.id))

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
	ax.clear()
	ax.set_aspect("equal")
	ax.set_title("Dist. Cov. Experiment")
	ax.set_xlim(lims[0] - 5., lims[2] + 5.)
	ax.set_ylim(lims[1] - 5., lims[3] + 5.)

	if globals()["region_patch"] is not None:
		ax.add_patch(globals()["region_patch"])

	for _, obs_patch in globals()["obstacle_patches"].items():
		ax.add_patch(obs_patch)

	for aid, state in S.items():
		# State visualization
		pos, vel, hdg, goal = state['pos'], state['vel'], state['hdg'], state['goal']
		robot_color = globals()['__COLORS'][aid]
		ax.quiver(pos[0], pos[1], np.cos(hdg), np.sin(hdg), color=robot_color)
		ax.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))
		ax.add_artist(plt.Circle(tuple(goal), 1., color=robot_color))
		x_hist, y_hist = zip(*(globals()["motion_history"][aid]))
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

def customSigIntHandler(signum, frame):
	# for aid, vlv_history in globals()["all_vlv_history"].items():
	# 	with open("Agent{}_VLV.json".format(aid), "w") as H:
	# 		json.dump(vlv_history, H, indent=4)

	# 	print("Dumped Agent {} VLV history.".format(aid))

	with open("all_vlv_history.json", "w") as H:
		json.dump(globals()["all_vlv_history"], H, indent=4)

	print("Dumped all VLV history.")

if __name__ == "__main__":
	agent_count = int(sys.argv[1])
	rospy.init_node("monitor", anonymous=False, disable_signals=True)

	signal.signal(signal.SIGINT, customSigIntHandler)

	states_sub = rospy.Subscriber("/states", AgentState, state_cb, queue_size=20)
	vpoly_sub = rospy.Subscriber("/visibility_polys", Polygon, vpoly_cb, queue_size=20)
	cvx_vor_sub = rospy.Subscriber("/convex_voronoi", Polygon, voronoi_cb, queue_size=20)
	vlv_poly_sub = rospy.Subscriber("/visibility_limited_voronoi", HistoryStep, vlv_poly_cb, queue_size=20)

	vp_vis_focus_service = rospy.Service("/set_vp_vis_focus", SetId, handle_vp_focus)
	vis_focus_level_service = rospy.Service("/set_vis_focus_level", SetId, handle_vis_level)
	print_instantaneous_mass = rospy.Service("/print_instantaneous_mass", PrintMass, handle_print_mass)

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

	rand_positions = get_random_positions(exp_region, exp_obstacles, limits, agent_count)

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

	figure = plt.figure()
	exp_ax = figure.add_subplot(1, 1, 1)
	ani_func = animation.FuncAnimation(figure, 
									   partial(animate_experiment, 
									   		   ax=exp_ax, 
									   		   lims=limits, 
									   		   S=all_states, 
									   		   VP=all_vpolygons, 
									   		   VLV=all_vl_voronoi), 
									   interval=100)

	rospy.loginfo("Experiment will be visualized, now...")
	plt.show(block=True)

	rospy.spin()