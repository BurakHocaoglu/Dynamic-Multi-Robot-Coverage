#ifndef TASK_PLANNING_AGENT_H
#define TASK_PLANNING_AGENT_H

#include <ros/ros.h>
#include <vector>
#include <utility>
#include <cmath>

#include "task_planning/ControllerConnector.h"

namespace task_planning {

    class Agent {
        public:
            Agent();
            ~Agent();

            void broadcast_flight_state();
            void plan();
            void solve();
            void execute();

        private:
            ros::NodeHandle node_handle_;

            std::shared_ptr<ControllerConnector> controller_;
            std::unordered_map<uint8_t, FlightState_v1> neighbours_;

            // AgentMode a_mode_;
    };

}

#endif // TASK_PLANNING_AGENT_H