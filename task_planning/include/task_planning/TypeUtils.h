#ifndef TASK_PLANNING_TYPE_UTILS_H
#define TASK_PLANNING_TYPE_UTILS_H

#include <string>

namespace task_planning {

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
    };

}

#endif // TASK_PLANNING_TYPE_UTILS_H