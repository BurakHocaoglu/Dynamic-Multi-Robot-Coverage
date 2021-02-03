#!/usr/bin/env python
# -*- coding: utf-8 -*-

from uas import *

if __name__ == '__main__':
	uid = int(sys.argv[1])
	is_voronoi = (sys.argv[2] == "true")
	rospy.init_node('uas{}'.format(uid), anonymous=False)
	uas = UAS(uid)
	# rospy.logwarn('UAS {} has grid map shape {}.'.format(uas.uid, uas.local_grid_map.shape))

	A_cell_converged, b_cell_converged = None, None
	instance_after_init = time.time()
	rate = rospy.Rate(uas.args['rate'])
	while not rospy.is_shutdown():
		uas.broadcast_state()

		if time.time() - instance_after_init > 5.:
			v_step = np.zeros(2)

			if is_voronoi:
				# if uas.coverage_state == 0:
				# 	if not uas.all_converged():
				# 		# A_iq, b_iq = uas.compute_bisectors()
				# 		A_iq, b_iq = uas.compute_bisectors2()
				# 		v_step = uas.solve_step()

				# 	else:
				# 		print("UAS {} has converged! Transitioning phase 2...".format(uid))

				# 		uas.coverage_state = 1
				# 		A_cell_converged, b_cell_converged = uas.compute_voronoi_constraints()

				# elif uas.coverage_state == 1:
				# 	v_step = uas.solve_step3(A_cell_converged, b_cell_converged)

				A_iq, b_iq = uas.compute_bisectors2()
				v_step = uas.solve_step_by_force()

			else:
				v_step = uas.solve_step3()

			uas.execute_step(v_step)
			
			# rospy.loginfo("UAS {} - Remaining time: {} secs.".format(uid, rate.remaining().secs))

		rate.sleep()

	rospy.spin()