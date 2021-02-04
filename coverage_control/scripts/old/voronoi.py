#!/usr/bin/env python

from math_utils import *

COLORS = [(0.0, 0.0, 0.0), 
		  (0.99, 0.0, 0.0), 
		  (0.0, 0.99, 0.0), 
		  (0.0, 0.0, 0.99), 
		  (0.99, 0.99, 0.0), 
		  (0.99, 0.0, 0.99), 
		  (0.0, 0.99, 0.99)]

boundary_vertices = []
boundary_segments = []
obstacle_vertices = dict()
obstacle_segments = []

class StateBuffer:

	def __init__(self):
		self.buffers = dict()

	def getState(self, name):
		return self.buffers[name]

	def getAllStates(self):
		return dict(self.buffers)

	def updateState(self, name, s):
		self.buffers[name] = s

class Agent:

	def __init__(self, name, init, goal, vmax):
		self.name = name
		self.aid = int(self.name[3:])
		self.move_thread = threading.Thread(name="{}_move".format(self.name), target=self.move)

		self.terminate = False
		self.dt = 0.1
		self.vmax = vmax
		self.vmin = 0.5
		self.velocity = np.zeros(2)
		self.position = np.array(init, dtype=float)
		self.voronoi_cell = []
		self.color = globals()['COLORS'][int(self.name[3:])]

		self.neighbours = dict()
		self.xhistory = []
		self.yhistory = []
		self.goal = np.array(goal, dtype=float)
		self.goal_change = 10.
		self.converged = False

		self.state = {'pos': self.position, 'vel': self.velocity, 'end': False}
		self.advertiseState()

	def initialize(self):
		self.move_thread.start()

	def setGoal(self, g):
		self.goal_change = np.linalg.norm(g - self.goal)
		self.converged = self.goal_change <= 0.1
		self.goal = np.array(g, dtype=float)

	def hasReachedGoal(self):
		return np.linalg.norm(self.goal - self.state['pos']) <= 0.1 and self.converged

	def computeBisectors(self):
		constraints, values = [], []
		bisectors = []

		##################################################################################
		#### Construct voronoi dividers ##################################################
		for nbid, st in self.neighbours.items():
			normal = st['pos'] - self.position
			middle = (st['pos'] + self.position) * 0.5
			constraints.append(normal)
			values.append(middle.dot(normal))
			bisectors.append((normal, middle, "D_{}{}".format(self.aid, nbid)))
		##################################################################################

		A_cell = np.array(constraints, dtype=float)
		b_cell = np.array(values, dtype=float)
		cell_graph = VGraph()
		self.voronoi_cell = []

		##################################################################################
		#### Voronoi dividers vs. Voronoi dividers #######################################
		for i in range(len(bisectors) - 1):
			n_i, m_i, name_i = bisectors[i]
			d_i = m_i.dot(n_i)

			for j in range(i + 1, len(bisectors)):
				n_j, m_j, name_j = bisectors[j]
				d_j = m_j.dot(n_j)

				try:
					A_ = np.array([n_i.round(2), n_j.round(2)], dtype=float)
					b_ = np.array([d_i.round(2), d_j.round(2)], dtype=float)
					p = np.linalg.solve(A_, b_).round(2)

				except np.linalg.LinAlgError:
					continue

				except:
					print(traceback.format_exc())
					continue

				if (is_point_feasible(A_cell, b_cell, p) and 
					is_point_valid(globals()['boundary_vertices'], p, globals()['obstacle_vertices'])):
					cell_graph.add_edge(name_i, name_j, VEdge("P_{}_{}".format(name_i, name_j), p))
		##################################################################################

		##################################################################################
		# Adding boundary vertices to cell graph, if necessary ###########################
		for i in range(len(globals()['boundary_vertices'])):
			j = (i + 1) % len(globals()['boundary_vertices'])
			k = (i - 1 + len(globals()['boundary_vertices'])) % len(globals()['boundary_vertices'])

			if is_point_feasible(A_cell, b_cell, globals()['boundary_vertices'][i]):
				first, second = "B_{}{}".format(k, i), "B_{}{}".format(i, j)
				cell_graph.add_edge(first, second, VEdge("BND_{}".format(i), globals()['boundary_vertices'][i]))
		##################################################################################

		self.voronoi_cell = cell_graph.traverse()

	def solveStep(self):
		v_next = self.state['vel']

		if len(self.voronoi_cell) >= 3:
			self.voronoi_cell.append(self.voronoi_cell[0])
			self.setGoal(self.getCentroid())
			v_next = self.goal - self.state['pos']
			_norm = np.linalg.norm(v_next)

			if _norm > self.vmax:
				v_next *= self.vmax / np.linalg.norm(v_next)

			return v_next

		print('Agent {} stopped momentarily.'.format(self.name))
		return np.zeros(2)

	def doStep(self, v_next):
		x_, y_ = self.state['pos'][0], self.state['pos'][1]
		self.xhistory.append(x_)
		self.yhistory.append(y_)
		self.state['pos'] = self.state['pos'] + self.dt * v_next
		self.state['vel'] = v_next

	def updateNeighbours(self):
		for uav, st in globals()['buf'].buffers.items():
			if uav == self.name or st is None:
				continue

			self.neighbours[uav] = dict(st)

	def advertiseState(self):
		globals()['buf'].updateState(self.name, self.state)

	def stop(self):
		self.terminate = True

	def move(self):
		pre_flight_count = 20

		while not self.terminate:
			_start = time.time()

			self.advertiseState()
			self.updateNeighbours()

			if pre_flight_count < 1:
				self.computeBisectors()
				v_next = self.solveStep()
				self.doStep(v_next)

			else:
				pre_flight_count -= 1

			_elapsed = time.time() - _start
			if _elapsed >= self.dt:
				print('Agent {} failed hard real-time constraint.'.format(self.name))

			else:
				time.sleep(self.dt - _elapsed)

		self.state['end'] = True
		if self.hasReachedGoal():
			print("Agent {} has reached goal at {}".format(self.name, datetime.datetime.now()))

class Simulator:

	def __init__(self, pfile):
		self.xlim = (0, 0)
		self.ylim = (0, 0)
		self.count = 0
		self.agents = dict()
		self.agent_inits = dict()
		self.vmax = 0
		self.iteration = 0
		self.loadParams(pfile)

		self.terminate = False

		self.fig = plt.figure()
		self.ax = self.fig.add_subplot(1, 1, 1)
		self.ani = None

	def loadParams(self, pfile):
		params = None
		with open(pfile) as P:
			params = yaml.load(P)

		self.region = np.array(params['bounding_polygon'], dtype=float)
		self.obstacles = params['obstacles']

		##################################################################################
		#### Construct boundary segments #################################################
		region_boundary = []

		for i in range(len(self.region)):
			j = (i + 1) % len(self.region)
			middle = (self.region[j] + self.region[i]) * 0.5
			direction = self.region[j] - self.region[i]
			normal = np.array([-direction[1], direction[0]])
			region_boundary.append((normal, middle, "B_{}{}".format(i, j)))

		globals()['boundary_segments'] = region_boundary
		globals()['boundary_vertices'] = self.region
		##################################################################################

		##################################################################################
		#### Construct obstacle segments #################################################
		obstacle_boundary = []
		obs_vertices = dict()

		for hn, hv in self.obstacles.items():
			obs_vertices[hn] = np.array(hv, dtype=float)

			for i in range(len(hv)):
				j = (i + 1) % len(hv)
				middle = (hv[j] + hv[i]) * 0.5
				direction = hv[j] - hv[i]
				normal = np.array([-direction[1], direction[0]], dtype=float)
				obstacle_boundary.append((normal, middle, "H_{}_{}_{}".format(hn, i, j)))

		globals()['obstacle_segments'] = obstacle_boundary
		globals()['obstacle_vertices'] = obs_vertices
		##################################################################################

		xcoords, ycoords = zip(*(self.region))
		self.xlim = (min(xcoords), max(xcoords))
		self.ylim = (min(ycoords), max(ycoords))
		self.count = params['count']
		self.vmax = params['vmax']

		self.bounding_poly_plt = ptc.Polygon(self.region, color=(0, 0, 0), fill=False)

		self.create_generators()

	def create_generators(self):
		valid_samples, i = [], 0
		low_limit = [self.xlim[0], self.ylim[0]]
		high_limit = [self.xlim[1], self.ylim[1]]

		while i < self.count:
			p = np.random.uniform(low_limit, high_limit, (2,))

			if is_point_valid(self.region, p, self.obstacles):
				valid = True

				for sample in valid_samples:
					if np.linalg.norm(p - sample) <= 2.:
						valid = False
						break

				if valid:
					valid_samples.append(p)
					self.agent_inits[i] = p
					i += 1

		for i in range(self.count):
			self.agents[i] = Agent(name="uav{}".format(i), 
								   goal=np.zeros(2), 
								   init=self.agent_inits[i], 
								   vmax=self.vmax)

			print("Agent {} will start from {}.".format(i, self.agent_inits[i]))

	def isDone(self):
		return all([a.state['end'] for _, a in self.agents.items()])

	def animate_motion(self, i):
		self.ax.clear()
		self.ax.set_xlim(self.xlim[0] - 5, self.xlim[1] + 5)
		self.ax.set_ylim(self.ylim[0] - 5, self.ylim[1] + 5)
		self.iteration += 1

		for _, a in self.agents.items():
			pos = a.state['pos']
			vel = a.state['vel']
			angle = np.arctan2(vel[1], vel[0])
			circle = plt.Circle(tuple(pos), 0.5, color=a.color)
			self.ax.quiver(pos[0], pos[1], np.cos(angle), np.sin(angle), color=a.color)
			self.ax.add_artist(circle)
			self.ax.plot(a.xhistory, a.yhistory, color=a.color)
			self.ax.add_patch(self.bounding_poly_plt)

			polygon = a.voronoi_cell
			if len(polygon) < 3:
				continue

			poly = plt.Polygon(polygon, alpha=0.4, color=a.color)
			self.ax.add_patch(poly)

	def stop(self):
		self.terminate = True

	def run(self):
		print("Run starts at {}".format(datetime.datetime.now()))

		for _, a in self.agents.items():
			a.initialize()

		self.ani = animation.FuncAnimation(self.fig, self.animate_motion, interval=100)
		#self.ani = animation.FuncAnimation(self.fig, self.animate_motion, frames=3000, interval=100)
		#self.ani.save('lloyd_{self.count}_uav.mp4', writer='ffmpeg', fps=30)
		plt.show()

		while not self.terminate and not self.isDone():
			time.sleep(1)

		for _, a in self.agents.items():
			a.stop()

		print("Run done at {}".format(datetime.datetime.now()))

def ctrl_c_handler(signum, frame):
	globals()['sim'].stop()
	print('Closing...')

if __name__ == '__main__':
	buf = StateBuffer()
	sim = Simulator(sys.argv[1])
	signal.signal(signal.SIGINT, ctrl_c_handler)
	sim.run()