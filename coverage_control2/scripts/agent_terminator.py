#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import traceback

from std_srvs.srv import Trigger
from coverage_control2.srv import MultipleTermination

termination_clients = dict()

def handle_multiple_termination(req):
	try:
		for aid, client in globals()["termination_clients"].items():
			if aid in req.ids:
				client.wait_for_service()

				response = client()
				if response.success:
					rospy.loginfo("Killed Agent{}.".format(i + 1))

				else:
					rospy.logwarn("Failed to kill Agent{}.".format(i + 1))

		return True
	except Exception as e:
		# raise e
		print(traceback.format_exc())
		return False

if __name__ == "__main__":
	agent_count = int(sys.argv[1])

	rospy.init_node("agent_terminator", anonymous=False)

	multi_termination_srv = rospy.Service("/multi_terminate", MultipleTermination, handle_multiple_termination)

	for i in range(agent_count):
		termination_clients[i + 1] = rospy.ServiceProxy("/Agent{}/self_terminate".format(i + 1), Trigger)

	# rate = rospy.Rate(2)
	# while not rospy.is_shutdown():
	# 	pass
	# 	rate.sleep()

	rospy.spin()
