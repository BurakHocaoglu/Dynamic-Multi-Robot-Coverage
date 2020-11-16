#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <queue>
#include <tuple>
#include <string>
#include <utility>
#include <fstream>
#include <unordered_map>

#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>

#include <coverage_control/FlightState.h>

#include <Eigen/Dense>

// typedef Matrix<float, 3, 1> Vector3f;
typedef Matrix<double, 2, 1> Vector2d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, 4, 1> Vector4d;

enum class ControllerType {
	NONE = 0, // Direct control, for lightweight experiments
	PX4 = 1,
	ARDUCOPTER = 2
};

enum class SimulatorType {
	NONE = 0, // Plot simulation, for lightweight experiments
	RVIZ = 1,
	GAZEBO = 2,
	AIRSIM = 3
};

namespace coverage_control_utils {

template<std::string TKey, typename TValue>
using configs = std::unordered_map<TKey, TValue>;

}