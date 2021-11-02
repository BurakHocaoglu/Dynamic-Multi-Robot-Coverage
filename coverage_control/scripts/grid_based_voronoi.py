from utils import *

import traceback
import threading
import numpy as np
import multiprocessing as mp
import matplotlib.pyplot as plt

COLORS = [(0.99, 0., 0.), 
		  (0., 0.99, 0.), 
		  (0., 0., 0.99), 
		  (0.99, 0.99, 0.), 
		  (0.99, 0., 0.99), 
		  (0., 0.99, 0.99), 
		  (0., 0., 0.)]

def real_to_grid(p, offset, resolution):
	return (int(round((p[0] - offset[0]) * resolution)), 
			int(round((p[1] - offset[1]) * resolution)))

def grid_to_real(p, offset, resolution):
	return np.array([float(p[0]) / resolution + offset[0], 
					 float(p[1]) / resolution + offset[1]])

class Agent:

	def __init__(self, aid, pos):
		self.aid = aid
		self.pos = pos

agents = dict()
num_agents = 5
resolution = 1.
boundary = np.array([(-60., 60.), 
					 (-60., -60.), 
					 (60., -60.), 
					 (60., 60.)], dtype=float)

obstacles = {
	'obs1': np.array([(-20., 20.), 
					  (20., 8.), 
					  (-20., 0.)], dtype=float)
}

xcrd, ycrd = zip(*boundary)
west, east = min(xcrd), max(xcrd)
south, north = min(ycrd), max(ycrd)
GH = int(east - west)
GW = int(north - south)
offset = (west, south)
grid = np.zeros((GH, GW))

valid_samples, i = [], 0
lower_bound = [west + 1., south + 1.]
upper_bound = [east - 1., north - 1.]
while i < num_agents:
	p = np.random.uniform(lower_bound, upper_bound, (2,))

	valid = True
	for sample in valid_samples:
		if np.linalg.norm(p - sample) <= 2.:
			valid = False
			break

	if valid:
		agents[i] = Agent(i, p)
		valid_samples.append(p)
		i += 1

def assign(spid, grid, P, rStart, rEnd, cStart, cEnd, offset, resolution):
	# print("Process {} will work on ({} -> {}) - ({} -> {})".format(spid, rStart, rEnd, cStart, cEnd))

	for i in range(rStart, rEnd):
		for j in range(cStart, cEnd):
			gp = (i, j)
			p = grid_to_real(gp, offset, resolution)
			distances = np.linalg.norm(P - p, axis=1)
			idx = np.argmin(distances)

			# if spid == 0:
			# 	# print("Process {} has agent positions for point {}: {}".format(spid, p, P))
			# 	print("Process {} has agent distances for point {}: {}".format(spid, p, distances))
			# 	print("Process {} has agent computed index for point {}: {}".format(spid, p, idx))

			grid[i, j] = idx

positions = np.array([a.pos for _, a in agents.items()], dtype=float)
print("Positions (Real - Grid), of shape {}:".format(positions.shape))
for p in positions:
	print("{} - {}".format(p, real_to_grid(p, offset, resolution)))

try:
	block_size = 32
	block_height = int(GH / block_size)
	block_width = int(GW / block_size)

	processes = []
	k = 0
	# for i in range(block_height):
	# 	for j in range(block_width):
	# 		rStart = i * block_size
	# 		rEnd = rStart + block_size
	# 		cStart = j * block_size
	# 		cEnd = cStart + block_size
	# 		pargs = (k, grid, positions, rStart, rEnd, cStart, cEnd, offset, resolution)
	# 		# processes.append(mp.Process(name="P{}".format(k), target=assign, args=pargs))
	# 		processes.append(threading.Thread(name="T{}".format(k), target=assign, args=pargs))
	# 		k += 1

	for p in processes:
		p.start()

	for i in range(GH):
		for j in range(GW):
			gp = (i, j)
			p = grid_to_real(gp, offset, resolution)

			if is_point_valid(boundary, p, obstacles):
				distances = np.linalg.norm(positions - p, axis=1)
				k = np.argmin(distances)
				grid[gp] = k

			else:
				grid[gp] = -1

	# for _, a in agents.items():
	# 	p = real_to_grid(a.pos, offset, resolution)
	# 	grid[p] = a.aid

	# print("Final Grid:")
	# for i in range(GH):
	# 	print(grid[i])

	for p in processes:
		p.join()

	image = np.zeros((GH, GW, 3))
	for i in range(GH):
		for j in range(GW):
			image[j, i, :] = np.array(COLORS[int(grid[i, j])])

	plt.imshow(image)
	plt.show()

except Exception as e:
	print(traceback.format_exc())