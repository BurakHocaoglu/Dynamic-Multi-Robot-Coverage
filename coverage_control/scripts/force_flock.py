import time
import signal
import threading
import traceback
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.animation as animation
from functools import partial

all_agents = dict()
all_states = dict()

__COLORS = [(0,0,0), (0.99,0,0), (0,0.99,0), (0,0,0.99), (0.99,0.99,0), (0.99,0,0.99),
			(0,0.99,0.99), (0.99,0,0.5), (0.99,0.5,0), (0.,0.99,0.5), (0.5,0.5,0.5)]

class Agent:

	def __init__(self, aid, pos, sTable):
		self.aid = aid
		self.p = pos
		self.v = np.zeros(2)
		self.sTable = sTable
		self.move_thread = threading.Thread(name="ThMove_{}".format(self.aid), target=self.move)
		self.terminate = False
		self.dt = 0.1
		self.a = 0.2
		self.b = 0.2
		self.vmax = 2.0
		self.r_safe = 2.0
		print("Agent {} is ready at {}".format(self.aid, self.p))

	def start(self):
		self.move_thread.start()

	def cancel(self):
		self.terminate = True

	def broadcast(self):
		self.sTable[self.aid] = {'pos': self.p, 'vel': self.v}

	def repulsion(self):
		force = np.zeros(2)

		for aid, state in self.sTable.items():
			if aid == self.aid:
				continue

			direction = self.p - state['pos']
			norm = np.linalg.norm(direction)
			unit_repulsion = direction #/ norm
			force += unit_repulsion * self.b / (norm - 2. * self.r_safe) ** 2

		return force

	def attraction(self):
		force = np.zeros(2)

		for aid, state in self.sTable.items():
			if aid == self.aid:
				continue

			direction = state['pos'] - self.p
			norm = np.linalg.norm(direction)
			unit_repulsion = direction #/ norm
			force += unit_repulsion * self.a / norm

		return force

	def step(self):
		v_norm = np.linalg.norm(self.v)

		if v_norm < 1e-2:
			print("Agent {} is too fucking slow!".format(self.aid))

		# else:
		# 	print("Agent {} has speed {}".format(self.aid, v_norm))

		self.p += self.v * self.dt

	def move(self):
		pre_stamp = time.time()

		while not self.terminate:
			_start = time.time()

			self.broadcast()

			if time.time() - pre_stamp > 3.0:
				self.v = np.zeros(2)

				F_rep = self.repulsion()
				F_att = self.attraction()
				F_sum = F_rep + F_att

				norm = np.linalg.norm(F_sum)
				if norm > self.vmax:
					F_sum *= self.vmax / norm

				self.v = F_sum
				self.step()

			_elapsed = time.time() - _start
			if _elapsed >= self.dt:
				print("Agent {} missed real-time constraint!".format(self.aid))

			else:
				time.sleep(self.dt - _elapsed)

		print("ThMove_{} has terminated!".format(self.aid))

def get_random_positions(lims, count):
	valid_samples, i = [], 0
	low_limit = [lims[0] + 1., lims[1] + 1.]
	high_limit = [lims[2] - 1., lims[3] - 1.]

	while i < count:
		p = np.random.uniform(low_limit, high_limit, (2,))
		valid = True

		for sample in valid_samples:
			if np.linalg.norm(p - sample) <= 2.:
				valid = False
				break

		if valid:
			valid_samples.append(p)
			i += 1

	return valid_samples

def animate_motion(i, ax, bnd, lims, states):
	ax.clear()
	ax.set_xlim(lims[0] - 5., lims[2] + 5.)
	ax.set_ylim(lims[1] - 5., lims[3] + 5.)
	ax.set_title("Flocking")
	ax.set_aspect("equal")

	if bnd is not None:
		ax.add_patch(plt.Polygon(bnd, color=(0., 0., 0.), fill=False))

	for aid, state in states.items():
		pos, vel = state['pos'], state['vel']
		heading = np.arctan2(vel[1], vel[0])
		robot_color = globals()['__COLORS'][aid + 1]
		ax.quiver(pos[0], pos[1], np.cos(heading), np.sin(heading), color=robot_color, linewidth=1.)
		ax.add_artist(plt.Circle(tuple(pos), 1., color=robot_color))

def ctrl_c_handler(signum, frame):
    print('Closing...')

    for _, agent in globals()['all_agents'].items():
    	agent.cancel()
    	agent.move_thread.join()

if __name__ == "__main__":
	signal.signal(signal.SIGINT, ctrl_c_handler)

	count = 10
	boundary = np.array([(-40., 40.), 
						 (-40., -40.), 
						 (40., -40.), 
						 (40., 40.)], dtype=float)

	x_comps, y_comps = zip(*boundary)
	xmin, xmax = min(x_comps), max(x_comps)
	ymin, ymax = min(y_comps), max(y_comps)
	limit = (xmin, ymin, xmax, ymax)

	positions = get_random_positions(limit, count)

	for i in range(count):
		all_agents[i] = Agent(i, positions[i], all_states)

	figure = plt.figure()
	axes = figure.add_subplot(1, 1, 1)
	animation_func = animation.FuncAnimation(figure, partial(animate_motion, 
															    ax=axes, 
																bnd=boundary, 
																lims=limit,
																states=all_states), interval=100)

	for _, agent in all_agents.items():
		agent.start()

	plt.show()