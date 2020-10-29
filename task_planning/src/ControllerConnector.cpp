#include "task_planning/ControllerConnector.h"

namespace task_planning {

    ControllerConnector::ControllerConnector() {
        type_ = ControllerType.CNTR_NAIVE;
        ready_ = false;
    }

    ControllerConnector::~ControllerConnector() {
        // Nothing for now ...
    }

    void ControllerConnector::initialize(ros::NodeHandle& n) {
        node_handle_ = n;
    }

    void ControllerConnector::set_controller_mode(std::string& mode) {
        cntr_state_.mode = mode;
    }

    void ControllerConnector::set_controller_stream_rate(float rate) {
        //
    }

    void ControllerConnector::set_controller_rtl_altitude(float) {
        //
    }

    void ControllerConnector::set_controller_land_altitude(float) {
        //
    }

    bool ControllerConnector::arm(bool value) {
        if (cntr_state_.pz >)
    }

    bool ControllerConnector::takeoff(float altitude) {
        //
    }

    bool ControllerConnector::land() {
        //
    }

}