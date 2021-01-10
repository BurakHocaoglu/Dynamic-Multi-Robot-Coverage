import numpy as np
import matplotlib.pyplot as plt

# num_boundary = 5
# num_boundary = 6
# num_boundary = 7
num_boundary = 10

star_vertices = [[-60., 0.], 
				 [0., 43.6], 
				 [60., 0.], 
				 [37.1, 70.5], 
				 [97.1, 114.1], 
				 [22.9, 114.1], 
				 [0., 184.7], 
				 [-24.6, 114.1], 
				 [-97.1, 114.1], 
				 [-37.6, 69.], 
				 [-20., 20.], 
				 [-20., -20.], 
				 [20., -20.], 
				 [20., 20.]]

# vertices = [np.array([0., 12.]), 
# 			np.array([-7.53, 4.94]), 
# 			np.array([-3.1, -4.4]), 
# 			np.array([7.1, -3.1]), 
# 			np.array([9., 7.]), 
# 			np.array([0.9, 7.]), 
# 			np.array([0.9, 3.2]), 
# 			np.array([4.6, 3.2]), 
# 			np.array([4.6, 7.])]

# vertices = [np.array([-20., 100.]), 
# 			np.array([-80., 40.]), 
# 			np.array([-60., -40.]), 
# 			np.array([-20., -20.]), 
# 			np.array([40., -60.]), 
# 			np.array([60., 60.])]

# vertices = [np.array([-20., 100.]), 
# 			np.array([-80., 60.]), 
# 			np.array([-40., 20.]), 
# 			np.array([-80., -40.]), 
# 			np.array([40., 20.]), 
# 			np.array([100., 40.]), 
# 			np.array([20., 60.])]

vertices = [np.array(v, dtype=float) for v in star_vertices]

# position = np.array([-2.96, 5.02])

boundary_vertices = list(vertices[:num_boundary])
hole_vertices = list(vertices[num_boundary:])

boundary_centroid = np.mean(boundary_vertices, axis=0)
hole_vertices = map(lambda v: v + boundary_centroid, hole_vertices)

boundary_polygon = plt.Polygon(boundary_vertices, alpha=0.9, color=(0., 0., 0.99), fill=False)
hole_polygon = plt.Polygon(hole_vertices, alpha=0.9, color=(0.99, 0., 0.), fill=False)

figure = plt.figure()
ax = figure.add_subplot(1, 1, 1)

xs, ys = zip(*vertices)
ax.set_xlim(min(xs) - 10., max(xs) + 10.)
ax.set_ylim(min(ys) - 10., max(ys) + 10.)
ax.set_aspect('equal')

ax.add_patch(boundary_polygon)
ax.add_patch(hole_polygon)

# agent = plt.Circle(tuple(position), 0.3, color=(0.99, 0.45, 0.))
# ax.add_patch(agent)

boundary_vertices.append(boundary_vertices[0])
hole_vertices.append(hole_vertices[0])

print("Boundary normals:")
for i in range(len(boundary_vertices) - 1):
	mid = (boundary_vertices[i + 1] + boundary_vertices[i]) * 0.5
	direction = boundary_vertices[i + 1] - boundary_vertices[i]
	vector = np.array([-direction[1], direction[0]])
	heading = np.arctan2(vector[1], vector[0])
	ax.quiver(mid[0], mid[1], np.cos(heading), np.sin(heading), color=(0.99, 0., 0.99))
	ax.add_patch(plt.Circle(tuple(mid), 0.1, color=(0., 0., 0.)))
	print("\t{}".format(vector))

print("Hole vectors:")
for i in range(len(hole_vertices) - 1):
	mid = (hole_vertices[i + 1] + hole_vertices[i]) * 0.5
	direction = hole_vertices[i + 1] - hole_vertices[i]
	vector = np.array([direction[1], -direction[0]])
	heading = np.arctan2(vector[1], vector[0])
	ax.quiver(mid[0], mid[1], np.cos(heading), np.sin(heading), color=(0., 0.99, 0.))
	ax.add_patch(plt.Circle(tuple(mid), 0.1, color=(0., 0., 0.)))
	print("\t{}".format(vector))

plt.show()