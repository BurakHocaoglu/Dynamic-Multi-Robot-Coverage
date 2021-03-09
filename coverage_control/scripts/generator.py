# from collections import OrderedDict

class G:

	def __init__(self, name=""):
		self.name = name
		self.last = None
		self.neighbours = dict()
		self.order = dict()
		self.vTable = globals()['visited']
		self.vTable[self.name] = False
		self.direction = True
		self.count = 0

	def add_neighbour(self, nb):
		if self.neighbours.get(nb.name) is None:
			self.neighbours[nb.name] = nb

	def sort(self):
		order_ = self.neighbours.items()
		order_.reverse()

		for i in range(len(order_)):
			k = (i - 1 + len(order_)) % len(order_)
			j = (i + 1) % len(order_)

			prev_edge = order_[k]
			from_edge = order_[i]
			to_edge = order[j]

			self.order[from_edge[0]] = (prev_edge[0], to_edge[0])

		self.vTable[self.name] = True

	def all_visited(self):
		for nb, nbv in self.vTable.items():
			if not nbv:
				return False

		return True

	def switch(self):
		self.direction = not self.direction

	def should_switch(self):
		pass

	def __iter__(self):
		return self

	def __next__(self):
		if self.order is None or len(self.order) == 0:
			raise StopIteration

		if self.all_visited():
			raise StopIteration

		if self.last is None:
			self.last = self.order.keys()[0]

		# previous = self.order.get(self.last)
		# if previous is not None and self.vTable.get(previous[1]):
		# 	self.switch()

		result = self.should_switch()

		if result is not None:
			self.vTable[self.order[self.last]] = True
			return self.order[self.last]

		else:
			raise StopIteration

	# def __next__(self):
	# 	## DIRECTION SWITCH
	# 	if self.last == len(self.order):
	# 		# switched = True
	# 		self.direction = False
	# 		self.last -= 1

	# 		while self.vTable.get(self.order[self.last - 1]):
	# 			self.last -= 1

	# 	elif self.last == -1:
	# 		# switched = True
	# 		self.direction = True
	# 		self.last += 1

	# 		while self.vTable.get(self.order[self.last + 1]):
	# 			self.last += 1

	# 	else:
	# 		self.apply_direction()


	# 	if self.count == 3:
	# 		self.last = 9

	def next(self):
		return self.__next__()

if __name__ == "__main__":
	visited = dict()

	gA = G("A")

	objects = [G("G_{}".format(i)) for i in range(10)]
	for obj in objects:
		gA.add_neighbour(obj)

	gA.sort()

	for g in gA:
		print('Visited: {}'.format(g))