#include "coverage_control2/coverage_agent.h"

int main(int argc, char **argv) {
	uint8_t aid = std::stoi(argv[1]);
	std::string node_name = "Agent" + std::string(argv[1]);

	ros::init(argc, argv, node_name.c_str());
	ros::NodeHandle node;

	Agent agent(node, node_name, aid);

	DebugLogConfig dcfg;

	float frequency;
	std::string centroid_alg;
	MotionParameters m_params;
	SensingParameters s_params;
	BehaviourSettings b_settings;

	std::map<std::string, bool> behaviours;
	std::map<std::string, float> motion_params, sensing_params;

	Polygon_2 exp_region;
	Polygon_2_Array exp_obstacles;
	XmlRpc::XmlRpcValue coverage_boundary, coverage_obstacles;

	if (node.getParam("/coverage_boundary", coverage_boundary)) {
		// ROS_ASSERT(coverage_boundary.getType() == XmlRpc::XmlRpcValue::TypeArray);

		create_poly_from_raw(coverage_boundary, exp_region, true);

		if (exp_region.is_clockwise_oriented()) 
			exp_region.reverse_orientation();

		ROS_INFO("Retrieved coverage boundary.");
	} else {
		ROS_ERROR("Could not retrieve coverage boundary!");
		return 0;
	}

	if (node.getParam("/coverage_obstacles", coverage_obstacles)) {
		// ROS_ASSERT(coverage_boundary.getType() == XmlRpc::XmlRpcValue::TypeArray);

		auto itr = coverage_obstacles.begin();

		for (; itr != coverage_obstacles.end(); itr++) {
			Polygon_2 obs_i;
			create_poly_from_raw(itr->second, obs_i);

			if (obs_i.is_counterclockwise_oriented()) 
				obs_i.reverse_orientation();

			exp_obstacles.push_back(obs_i);
		}

		ROS_INFO("Retrieved coverage obstacles.");
	} else {
		ROS_ERROR("Could not retrieve coverage obstacles!");
		return 0;
	}

	agent.set_task_region_from_raw(exp_region, exp_obstacles);

	if (node.getParam("/motion_params", motion_params)) {
		m_params.delta_t = motion_params["delta_t"];
		frequency = 1. / m_params.delta_t;

		m_params.attraction_const = motion_params["attraction_const"];
		m_params.repulsion_const = motion_params["repulsion_const"];
		m_params.vmax = motion_params["vmax"];
		m_params.umax = motion_params["umax"];
		m_params.wmax = motion_params["wmax"];
		m_params.physical_radius = motion_params["physical_radius"];
		m_params.safe_radius = motion_params["safe_radius"];
		m_params.K_linear = motion_params["K_linear"];
		m_params.K_angular = motion_params["K_angular"];
		m_params.K_repulsion = motion_params["K_repulsion"];
		m_params.K_attraction = motion_params["K_attraction"];
		m_params.K_goal = motion_params["K_goal"];

		agent.set_motion_params(m_params);
		ROS_INFO("%s finished motion parameter setup.", node_name.c_str());
	} else {
		ROS_ERROR("Could not load motion params!");
		return 0;
	}

	if (node.getParam("/sensing_params", sensing_params)) {
		s_params.sigma_local = sensing_params["sigma_local"];
		s_params.sense_radius = sensing_params["sense_radius"];
		s_params.hfov_range = deg2rad(sensing_params["hfov_range_deg"]);

		agent.set_sensing_params(s_params);
		ROS_INFO("%s finished sensing parameter setup.", node_name.c_str());
	} else {
		ROS_ERROR("Could not load sensing params!");
		return 0;
	}

	if (node.getParam("/centroid_alg", centroid_alg)) {
		b_settings.centroid_alg = getAlgFromString(centroid_alg);
	} else {
		ROS_ERROR("Could not load behaviour settings!");
		return 0;
	}

	if (node.getParam("/behaviours", behaviours)) {
		b_settings.stationary = behaviours["stationary"];
		b_settings.limited_sensing = behaviours["limited_sensing"];
		b_settings.flocking = behaviours["flocking"];
		b_settings.planning = behaviours["planning"];
		b_settings.visibility = behaviours["visibility"];
		b_settings.voronoi = behaviours["voronoi"];

		agent.set_behaviour_settings(b_settings);
	} else {
		ROS_ERROR("Could not load behaviour settings!");
		return 0;
	}

	// std::map<std::string, bool> debug_settings;
	// if (node.getParam("/debug", debug_settings)) {
	// 	dcfg.exec_time_log = debug_settings["exec_time_log"]

	// 	ROS_INFO("%s retrieved debug logging configuration.", node_name.c_str());
	// } else {
	// 	// ...
	// }

	ros::Rate rate(frequency);
	ROS_INFO("%s will loop at %f Hz.", node_name.c_str(), frequency);

	ros::Time warmup_start = ros::Time::now();
	while (ros::ok() && !agent.ready()) {
		ros::spinOnce();
		rate.sleep();
	}

	ROS_INFO("%s has warmed up.", node_name.c_str());

	// std::chrono::high_resolution_clock::time_point cycle_start, cycle_end;
	while (ros::ok()) {
		// ros::Time cycle_start = ros::Time::now();
		// cycle_start = std::chrono::high_resolution_clock::now();

		agent.broadcast();
		agent.step();

		ros::spinOnce();
		// cycle_end = std::chrono::high_resolution_clock::now();
		// ROS_INFO("%s - Took: %.5f", node_name.c_str(), (ros::Time::now() - cycle_start).toSec());
		// ROS_INFO("%s - Took: %ld", node_name.c_str(), std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count());
		rate.sleep();
	}

	ROS_INFO("%s terminating...", node_name.c_str());

	return 0;
}