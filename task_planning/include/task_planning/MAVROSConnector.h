#ifndef TASK_PLANNING_MAVROS_CONNECTOR_H
#define TASK_PLANNING_MAVROS_CONNECTOR_H

#include "task_planning/ControllerConnector.h"
#include <unordered_map>

namespace task_planning {

    class MAVROSConnector : public ControllerConnector {
        public:
            MAVROSConnector();
            ~MAVROSConnector();

            void initialize(ros::NodeHandle&) override;

            void set_controller_mode(std::string&) override;
            void set_controller_stream_rate(float) override;
            void set_controller_rtl_altitude(float) override;
            void set_controller_land_altitude(float) override;

            bool arm(bool) override;
            bool takeoff(float) override;
            bool land() override;

            void velocity_pub(float, float, float) override;
            void position_pub(float, float, float, bool) override;

        private:
            std::unordered_map<std::string, ros::Publisher> publishers_;
            std::unordered_map<std::string, ros::Subscriber> subscribers_;
            std::unordered_map<std::string, ros::ServiceClient> clients_;
    };

}

#endif // TASK_PLANNING_MAVROS_CONNECTOR_H