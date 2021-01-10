import sys
import sympy
import traceback
import numpy as np
import matplotlib.pyplot as plt

class LineConstraint:

	def __init__(self, v1, v2):
		self.v1 = v1
		self.v2 = v2
		self.d = v2 - v1
		self.slope = np.arctan2(d[1], d[0])

class Map:

	def __init__(self, vertices=[], resolution=1., obstacles=dict()):
		self.resolution = resolution

		self.vertices = sympy.Polygon(*vertices)
		for v in vertices:
			# self.vertices
			pass

		self.obstacles = dict()
		for name, obs in obstacles:
			self.obstacles

		self.map = []
		self.create_map()

	def create_map(self):
		if len(self.vertices) < 2:
			pass

		xcoords, ycoords = zip(*(self.vertices))
		xmin, xmax = min(xcoords), max(xcoords)
		ymin, ymax = min(ycoords), max(ycoords)
		width = (xmax - xmin) / self.resolution
		height = (ymax - ymin) / self.resolution
		self.map = - np.ones((height, width))

		for obs in self.obstacles:
			pass

	pass

if __name__ == '__main__':
	pass