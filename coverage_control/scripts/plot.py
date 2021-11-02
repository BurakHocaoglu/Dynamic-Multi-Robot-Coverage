from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np

def on_segment(p1, q, p2):
	# print(f'Check with: {p1}, {q}, {p2}')
	return (q[0] <= max(p1[0], p2[0]) and q[0] >= min(p1[0], p2[0]) and 
			q[1] <= max(p1[1], p2[1]) and q[1] >= min(p1[1], p2[1]))

def orientation(p1, q, p2):
	value = (q[1] - p1[1]) * (p2[0] - q[0]) - (q[0] - p1[0]) * (p2[1] - q[1])

	if value == 0:
		return 0

	elif value > 0:
		return 1

	return 2

def do_intersect(p1, q1, p2, q2): 
	o1 = orientation(p1, q1, p2)
	o2 = orientation(p1, q1, q2)
	o3 = orientation(p2, q2, p1)
	o4 = orientation(p2, q2, q1)

	if o1 != o2 and o3 != o4:
		return True 

	if (o1 == 0 and on_segment(p1, p2, q1)):
		return True

	if (o2 == 0 and on_segment(p1, q2, q1)):
		return True

	if (o3 == 0 and on_segment(p2, p1, q2)):
		return True

	if (o4 == 0 and on_segment(p2, q1, q2)):
		return True

	return False

def is_inside(polygon, p): 
	if len(polygon) < 3:
		return False

	extreme = (np.inf, p[1]); 
	count, i = 0, 0

	while i < len(polygon):
		j = (i + 1) % len(polygon)

		# if i == len(polygon) - 1:
		# 	j = 0

		if do_intersect(polygon[i], polygon[j], p, extreme):
			if orientation(polygon[i], p, polygon[j]) == 0:
				# print(f'Check with: {polygon[i]}, {p}, {polygon[j]}')
				return on_segment(polygon[i], p, polygon[j]) 

		i += 1
		count += 1 

	return bool(count & 1)

def angle_in_2pi(v):
	return np.arctan2(v[1], v[0])

def angular_sort(reference, vertices):
	vectors = [p - reference for p in vertices]
	indexed_angles = [(angle_in_2pi(vectors[i]), i) for i in range(len(vectors))]
	indexed_angles.sort()
	return [vertices[i] for _, i in indexed_angles]

# _boundary = [[-10., -10.], [-10., 10.], [10., -10.], [10., 10.]]
_boundary = [[-10., -10.], [8., -10.], [8., -8.], [10., -8.], 
			 [10., 10.], [-8., 10.], [-8., 8.], [-10., 8.]]

np_boundary = [np.array(bv, dtype=float) for bv in _boundary]
sorted_boundary = angular_sort(np.mean(np_boundary, axis=0), np_boundary)

def dist_func_poly(p):
	global sorted_boundary

	if not is_inside(sorted_boundary, p):
		return 0

	dists = []
	augmented = sorted_boundary + list(sorted_boundary[0])

	for i in range(len(sorted_boundary)):
		mid = (augmented[i] + augmented[i + 1]) * 0.5
		direction = augmented[i + 1] - augmented[i]
		normal = np.array([-direction[1], direction[0]])
		p_vec = p - mid
		dists.append(np.dot(p_vec, normal) / np.linalg.norm(normal))

	return min(dists) ** 2

# dist_func = dist_func_poly
dist_func = lambda p: (4. - np.linalg.norm(p)) ** 2

fig = plt.figure()
ax = fig.gca(projection='3d')

# X = np.linspace(-5, 5, 100)
# Y = np.linspace(-5, 5, 100)
X = np.linspace(-10, 10, 100)
Y = np.linspace(-10, 10, 100)
XX, YY = np.meshgrid(X, Y)
Z = list(map(np.array, zip(XX.flatten(), YY.flatten())))

sigma = 2
dists_sq = np.array(list(map(dist_func, Z))).reshape(100, 100)

value_func = lambda a: np.e ** (- a / (2 * sigma ** 2))
distribution = np.e ** (- dists_sq / (2 * sigma ** 2))

poses = []
for i in range(10):
	pose = np.random.uniform([-10., -10.], [10., 10.], size=(2,))
	point = list(pose) + [value_func(dist_func(pose))]
	poses.append(point)

poses = np.array(poses, dtype=float)
print(poses.shape)
poses = poses.T
print(poses.shape)

# ax.scatter([pos[0]], [pos[1]], [pos[2]], color=(0., 0.99, 0.))
ax.scatter(poses[0, :], poses[1, :], poses[2, :], color=(0., 0.99, 0.))

# surf = ax.plot_surface(XX, YY, distribution, cmap=cm.coolwarm, linewidth=0, antialiased=False)
surf = ax.plot_surface(XX, YY, distribution, cmap=cm.coolwarm, alpha=0.7, linewidth=0, antialiased=True)

# ax.set_zlim(-2.01, 4.01)
ax.set_zlim(np.amin(distribution) - 1., np.amax(distribution) + 1.)
ax.zaxis.set_major_locator(LinearLocator(10))
ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

plt.show()