#ifndef TASK_PLANNING_TYPE_UTILS_H
#define TASK_PLANNING_TYPE_UTILS_H

#include <iostream>
#include <string>

#include <Eigen/Dense>

namespace task_planning {

    #define INFO_LOG_TAG "[INFO] "
    #define WARN_LOG_TAG "[WARN] "
    #define ERROR_LOG_TAG "[ERROR] "

    typedef Eigen::Matrix<double, 2, 1> Vector2d;
	typedef Eigen::Matrix<double, 3, 1> Vector3d;
	typedef Eigen::Matrix<double, 4, 1> Vector4d;

    enum ControllerType {
        CNTR_NAIVE=1,
        CNTR_MAVROS=2
    };

    enum SimulatorType {
        SIM_NAIVE=1,
        SIM_GAZEBO=2,
        SIM_AIRSIM=3
    };

    struct ControllerState {
        float px, py, pz; // Position
        float vx, vy, vz; // Velocity
        std::string mode;
        bool armed;
    };

    struct ControllerArgs {
        float stream_rate;
        float land_altitude;
        float rtl_altitude;
        float v_linear_max;
        float v_angular_max;
    };

    struct FlightState_v1 {
        uint8_t id;
        Vector3d position, velocity;
    };

    struct FlightState_v2 {
        uint8_t id;
        float x, y, z;
        float roll, pitch, yaw;
        float speed;
    };

}

#endif // TASK_PLANNING_TYPE_UTILS_H