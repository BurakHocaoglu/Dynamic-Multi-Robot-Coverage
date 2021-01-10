#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from utils import angular_sort

def midPointCircleDraw(x_centre, y_centre, r): 
	circle = []
	x, y = r, 0

	# circle.append((x + x_centre, y + y_centre))

	if r > 0:
		# circle.append((x + x_centre, y + y_centre))
		# circle.append((x + x_centre, -y + y_centre))
		# circle.append((y + x_centre, x + y_centre))
		# circle.append((-y + x_centre, x + y_centre))

		circle.append((x + x_centre, y + y_centre))
		circle.append((y + x_centre, x + y_centre))
		circle.append((-x + x_centre, -y + y_centre))
		circle.append((-y + x_centre, -x + y_centre))

	P = 1 - r

	while x > y:
		y += 1

		# Mid-point inside or on the perimeter
		if P <= 0:
			P = P + 2 * y + 1

		# Mid-point outside the perimeter
		else:
			x -= 1
			P = P + 2 * y - 2 * x + 1

		# All the perimeter points have
		# already been printed
		if x < y:
			break

		# Printing the generated point its reflection
		# in the other octants after translation
		circle.append((x + x_centre, y + y_centre))
		circle.append((-x + x_centre, y + y_centre))
		circle.append((x + x_centre, -y + y_centre))
		circle.append((-x + x_centre, -y + y_centre))

		# If the generated point on the line x = y then
		# the perimeter points have already been printed
		if x != y:
			circle.append((y + x_centre, x + y_centre))
			circle.append((-y + x_centre, x + y_centre))
			circle.append((y + x_centre, -x + y_centre))
			circle.append((-y + x_centre, -x + y_centre))

	return circle

if __name__ == '__main__':
	radius = 4.
	resolution = 1.

	cx, cy = [], []

	# angles = np.linspace(-np.pi, np.pi, 360)
	# for a in angles:
	# 	x_ = radius * np.cos(a)
	# 	y_ = radius * np.sin(a)
	# 	cx.append(x_)
	# 	cy.append(y_)
	# 	#print('({}, {})'.format(x_, y_))

	# curr_y = 0.
	# quad = 1
	# done = False
	# while True:
	# 	# print(curr_y)
	# 	cy.append(curr_y)
	# 	curr_x = np.sqrt(radius ** 2 - curr_y ** 2)

	# 	if quad == 1:
	# 		curr_y += resolution

	# 		if curr_y == radius:
	# 			# print('Quadrant 1 Done!')
	# 			quad = 2

	# 	elif quad == 2:
	# 		curr_y -= resolution
	# 		curr_x *= -1

	# 		if curr_y == 0:
	# 			# print('Quadrant 2 Done!')
	# 			quad = 3

	# 	elif quad == 3:
	# 		curr_y -= resolution
	# 		curr_x *= -1

	# 		if curr_y == -radius:
	# 			# print('Quadrant 3 Done!')
	# 			quad = 4

	# 	elif quad == 4:
	# 		curr_y += resolution

	# 		if curr_y == 0:
	# 			# print('Quadrant 4 Done!')
	# 			done = True

	# 	cx.append(curr_x)
	# 	# cy.append(curr_y)

	# 	if done:
	# 		break

	figure = plt.figure()
	ax = figure.add_subplot(1, 1, 1)

	circle = midPointCircleDraw(0, 0, 4)
	sorted_circle = angular_sort(np.mean(circle, axis=0), circle)
	cx, cy = zip(*sorted_circle)

	# sorted_circle = zip(cx, cy)

	ax.set_xlim(-5., 5.)
	ax.set_ylim(-5., 5.)

	ax.set_aspect('equal')
	ax.plot(cx, cy)
	ax.plot(cx, cy, 'bo')

	for v in sorted_circle:
		ax.add_artist(patches.Rectangle((v[0] - resolution / 2., v[1] - resolution / 2.), 
										resolution, 
										resolution, 
										color=(0.99, 0., 0.), 
										fill=True, 
										alpha=0.25))

	plt.show()