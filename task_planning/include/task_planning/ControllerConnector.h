#ifndef TASK_PLANNING_CONTROLLER_CONNECTOR_H
#define TASK_PLANNING_CONTROLLER_CONNECTOR_H

#include "task_planning/Connector.h"

namespace task_planning {

    class ControllerConnector : public Connector {
        public:
            ControllerConnector();
            ~ControllerConnector();

            void initialize(ros::NodeHandle&);

            virtual void set_controller_mode(std::string&);
            virtual void set_controller_stream_rate(float);
            virtual void set_controller_rtl_altitude(float);
            virtual void set_controller_land_altitude(float);

            virtual bool arm(bool);
            virtual bool takeoff(float);
            virtual bool land();

        private:
            ros::NodeHandle node_handle_;

            ControllerType type_;
            ControllerArgs args_;
            ControllerState state_;
            bool ready_;
    };

}

#endif // TASK_PLANNING_CONTROLLER_CONNECTOR_H