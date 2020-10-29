#include "task_planning/ControllerConnector.h"

namespace task_planning {

    ControllerConnector::ControllerConnector() {
        type_ = ControllerType.CNTR_NAIVE;
        ready_ = false;
        tag_ = "[CNTR-NAIVE] ";
    }

    ControllerConnector::~ControllerConnector() {
        // Nothing for now ...
    }

    void ControllerConnector::initialize(ros::NodeHandle& n) {
        node_handle_ = n;
    }

    void ControllerConnector::set_controller_mode(std::string& mode) {
        state_.mode = mode;
    }

    void ControllerConnector::set_controller_stream_rate(float rate) {
        args_.stream_rate = rate;
    }

    void ControllerConnector::set_controller_rtl_altitude(float altitude) {
        args_.rtl_altitude = altitude;
    }

    void ControllerConnector::set_controller_land_altitude(float altitude) {
        args_.land_altitude = altitude;
    }

    bool ControllerConnector::arm(bool value) {
        if (!value && state_.pz > args_.land_altitude + 0.5) {
            std::cout << tag_ << WARN_LOG_TAG << "cannot DISARM higher than land_altitude!" << std::endl;
            return false;
        }

        state_.armed = false;
    }

    bool ControllerConnector::takeoff(float altitude) {
        //
    }

    bool ControllerConnector::land() {
        //
    }

    void ControllerConnector::velocity_pub(float vx, float vy, float vz) {
        //
    }

    void ControllerConnector::position_pub(float px, float py, float pz, bool local=true) {
        //
    }

    // void ControllerConnector::log() {
    //     //
    // }

}