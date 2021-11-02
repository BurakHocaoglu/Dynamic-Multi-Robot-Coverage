#!/usr/bin/env python
# -*- coding: utf-8 -*-

from monitor import *

if __name__ == '__main__':
	rospy.init_node('monitor', anonymous=False)
	monitor = Monitor()

	rate = rospy.Rate(10)
	while (not rospy.is_shutdown()) and (not monitor.is_ready()):
		rate.sleep()

	monitor.valid = True
	rospy.loginfo('Monitor has valid data from all agents. Ready for inspection!')
	plt.show(block=True)

	# while not rospy.is_shutdown():
	# 	# plt.show(block=False)
	# 	# plt.show()
	# 	rate.sleep()

	rospy.spin()

	monitor.save_grid_image()