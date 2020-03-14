import numpy as np
from collections import deque

class Node:

	def __init__(self, name, t, point, normal=None):
		self.name = name
		self.type = t
		self.point = point
		self.normal = normal
		self.neighbours = dict()

class VGraph:

	def __init__(self):
		self.V = dict()

	def addVertexDirect(self, v):
		self.V[v.name] = v

	def addVertexIntersect(self, v):
		pass

	def addEdge(self, v1, v2):
		self.V[v1.name].neighbours[v2.name] = v2
		self.V[v2.name].neighbours[v1.name] = v1

	def traverse(self):
		if len(self.V) == 0:
			return []

		traversal = []
		stack = deque()
		rand_idx = np.random.random_integers(0, len(self.V), 1)[0]
		rand_key = self.V.keys()[rand_idx]
		stack.append(rand_key)
		visited = dict()
		visited[rand_key] = True

		while len(stack):
			next_key = stack.pop()
			if self.V[next_key].type == 'Point':
				traversal.append(self.V[next_key].point)

			nb_frontier = self.V[next_key].neighbours
			for name, _ in nb_frontier.items():
				if not visited.get(name):
					stack.append(name)
					visited[name] = True
					break

		return traversal