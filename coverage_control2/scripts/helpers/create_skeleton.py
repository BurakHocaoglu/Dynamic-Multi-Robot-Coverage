import numpy as np
import skgeom as sg

from scipy.special import softmax
from collections import deque

import matplotlib.pyplot as plt

rot90 = np.array([[np.cos(np.pi / 2.), np.sin(np.pi / 2.)],
				  [- np.sin(np.pi / 2.), np.cos(np.pi / 2.)]])

class PolyEdge:

	def __init__(self, p0, p1):
		self.p0 = p0
		self.p1 = p1
		self.m = (p0 + p1) * 0.5
		self.d = p1 - p0
		self.n = np.dot(globals()["rot90"], self.d)
		self.v = np.dot(self.n, self.m)

class EdgeList:

	def __init__(self, vertices=[]):
		self.edges = []

		for i in range(len(vertices)):
			j = (i + 1) % len(vertices)
			self.edges.append(PolyEdge(vertices[j], vertices[k]))

if __name__  == "__main__":
	outer_boundary = np.array([[-20., 100.], [-20., 0.], [20., 0.], [40., 20.], [60., 0.], 
								[100., 0.], [140., 30.], [120., 50.], [140., 70.], [140., 100.]])

	convexity = np.ones(outer_boundary.shape[0])
	convexity[3] = -1.
	convexity[7] = -1.



	poly_history = deque()
	poly_history.append(EdgeList(outer_boundary))

	for i in range(5):
		poly = poly_history.popleft()

		pass




	hist = [outer_boundary]
	for i in range(7):
		_poly = hist[-1]

		new_poly = []
		normal_mags = []
		normals = []
		for i in range(len(_poly)):
			j = (i - 1) % len(_poly)
			k = (i + 1) % len(_poly)
			vec1 = _poly[k] - _poly[i]
			vec2 = _poly[j] - _poly[i]
			vec1 /= np.linalg.norm(vec1)
			vec2 /= np.linalg.norm(vec2)
			normal = (vec1 + vec2) * 0.5
			normals.append(normal)
			norm = np.linalg.norm(normal)
			normal_mags.append(norm)
			# normal /= norm
			# new_poly.append(_poly[i] + 4 * normal * convexity[i])

		n_norm = softmax(normal_mags)
		for i in range(len(_poly)):
			# if normal_mags[i] <= 0.2:
			# 	continue

			new_poly.append(_poly[i] + normals[i] * convexity[i] / n_norm[i])

		hist.append(new_poly)

		plt.gcf().gca().add_patch(plt.Polygon(new_poly, 
			color=(0., 0., 1.), fill=False, alpha=0.5, lw=1.))

	# plt.savefig("skel_construct", bbox_inches="tight")

	plt.show()