from uas import *
import yaml

if __name__ == '__main__':
	uas_name = sys.argv[1]
	pass

	rospy.init_node()
	this_uas = UAS()

	rate = rospy.Rate(10.)
	while not rospy.is_shutdown():
		rate.sleep()