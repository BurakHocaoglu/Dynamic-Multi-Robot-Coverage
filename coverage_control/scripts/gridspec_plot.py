import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

if __name__ == '__main__':
	fig = plt.figure(constrained_layout=False)

	# gs = gridspec.GridSpec(ncols=3, nrows=2, figure=fig)

	widths = [2, 1, 1]
	heights = [2, 2]
	gs = gridspec.GridSpec(ncols=3, nrows=2, 
						   width_ratios=widths, 
						   height_ratios=heights, 
						   figure=fig)

	map_ax = fig.add_subplot(gs[:, 0])
	map_ax.set_title("Coverage Map")

	agent0_ax = fig.add_subplot(gs[0, 1])
	agent0_ax.set_title("UAS 0")

	agent1_ax = fig.add_subplot(gs[0, 2])
	agent1_ax.set_title("UAS 1")

	agent2_ax = fig.add_subplot(gs[1, 1])
	agent2_ax.set_title("UAS 2")

	agent3_ax = fig.add_subplot(gs[1, 2])
	agent3_ax.set_title("UAS 3")

	plt.show()