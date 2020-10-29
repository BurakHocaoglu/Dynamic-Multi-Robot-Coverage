#ifndef TASK_PLANNING_CONNECTOR_H
#define TASK_PLANNING_CONNECTOR_H

#include <ros/ros.h>
#include "task_planning/TypeUtils.h"

namespace task_planning {

    class Connector {
        public:
            virtual void initialize(ros::NodeHandle&) = 0;
            virtual ~Connector() = default;

        protected:
            Connector() {};
    };

}

#endif // TASK_PLANNING_CONNECTOR_H