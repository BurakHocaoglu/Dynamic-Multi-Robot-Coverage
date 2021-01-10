#!/usr/bin/env python
# -*- coding: utf-8 -*-

from uas import *

if __name__ == '__main__':
	uid = int(sys.argv[1])
	rospy.init_node('uas{}'.format(uid), anonymous=False)
	uas = UAS(uid, sys.argv[2])

	instance_after_init = time.time()
	rate = rospy.Rate(uas.args['rate'])
	while not rospy.is_shutdown():
		uas.broadcast_state()

		if time.time() - instance_after_init > 5.:
			A_iq, b_iq = uas.compute_bisectors()
			v_step = uas.solve_step()
			uas.execute_step(v_step)

		rate.sleep()

	rospy.spin()