#!/usr/bin/env python
# -*- coding: utf-8 -*-

from utils import *

from PIL import Image

from coverage_control.msg import FlightStateCompressed, SensingInfo, Grid#, GridFlightState
from coverage_control.srv import GetMap, PrintMap

# -1: Unexplored
# 0 : Explored free cell
# 1 : Explored obstacle cell

class MapManager:

	def __init__(self, name="", vertices=[], resolution=1., obstacles=dict()):
		self.map_name = name
		self.vertices = vertices
		self.resolution = resolution
		self.obstacles = obstacles
		self.map = None

		self.create_map()

		# self.sense_state_sub = rospy.Subscriber("/sense_states", GridFlightState, self.sense_state_cb)
		# self.flight_state_sub = rospy.Subscriber("/flight_states", FlightStateCompressed, self.flight_state_cb)
		# self.get_map_service = rospy.Service("/get_map", GetMap, self.handle_get_map)
		# self.print_map_service = rospy.Service("/print_map", PrintMap, self.handle_print_map)

	def create_map(self):
		if self.vertices is None or len(self.vertices) < 3:
			rospy.logwarn('Not enough vertices were provided for map creation!')
			sys.exit(-1)

		xcoords, ycoords = zip(*(self.vertices))
		xmin, xmax = min(xcoords), max(xcoords)
		ymin, ymax = min(ycoords), max(ycoords)
		width = (xmax - xmin) / self.resolution
		height = (ymax - ymin) / self.resolution
		self.map = - np.ones((height, width))

		for obs in self.obstacles:
			pass

	def set_cell(self, cell, value):
		# if cell is not inside
		self.map[cell] = value

	# def sense_state_cb(self, msg):
	# 	for sense in msg.covered_cells:
	# 		cell = (sense.grid_x, sense.grid_y)
	# 		# self.map[cell] = sense.data
	# 		self.set_cell(cell, sense.data)

	# def handle_get_map(self, req):
	# 	try:
	# 		H, W = self.map.shape
	# 		return True, Grid(rospy.Time.now(), H, W, tuple(self.map.flatten()))
	# 	except Exception as e:
	# 		print(traceback.format_exc())
	# 		return False, Grid()

	# def handle_print_map(self, req):
	# 	try:
	# 		pass
	# 		return True
	# 	except Exception as e:
	# 		print(traceback.format_exc())
	# 		return False