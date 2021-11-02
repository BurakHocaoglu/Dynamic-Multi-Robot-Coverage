#!/usr/bin/env python
# -*- coding: utf-8 -*-

from uas import *

if __name__ == '__main__':
	uid = int(sys.argv[1])
	is_voronoi = (sys.argv[2] == "true")
	rospy.init_node('uas{}'.format(uid), anonymous=False)
	uas = UAS(uid)

	A_cell_converged, b_cell_converged = None, None
	coverage_start = None
	first = True

	instance_after_init = time.time()
	rate = rospy.Rate(uas.args['rate'])
	while not rospy.is_shutdown():
		uas.broadcast_state()

		if time.time() - instance_after_init > 5.:
			v_step = np.zeros(2)

			if is_voronoi:
				if uas.coverage_state == 0:
					if not uas.all_converged():
						# A_iq, b_iq = uas.compute_bisectors()
						A_iq, b_iq = uas.compute_bisectors2()
						v_step = uas.solve_step()

					# else:
					# 	print("UAS {} has a cell mass of {}".format(uid, uas.get_converged_mass()))

					else:
						uas.coverage_state = 1
						print("UAS {} has transitioned from 0 to 1!".format(uid))
						coverage_start = uas.set_coverage_start_goal()
						uas.converged = False
						uas.converged_count = 0
						A_cell_converged, b_cell_converged = uas.compute_voronoi_constraints()

				elif uas.coverage_state == 1:
					if not uas.all_converged():
						# A_iq, b_iq = uas.compute_bisectors()
						# uas.set_coverage_start_goal()
						uas.set_goal(coverage_start)
						v_step = uas.solve_step(direct=True)

					else:
						uas.coverage_state = 2
						uas.heading = np.pi / 2.
						print("UAS {} has transitioned from 1 to 2!".format(uid))
						# A_cell_converged, b_cell_converged = uas.compute_voronoi_constraints()

				elif uas.coverage_state == 2:
					v_step = uas.solve_step3(A_cell_converged, b_cell_converged, first)
					first = False

				# A_iq, b_iq = uas.compute_bisectors()
				# v_step = uas.solve_step_by_force()

			else:
				v_step = uas.solve_step2()

			uas.execute_step(v_step)

		rate.sleep()

	rospy.spin()