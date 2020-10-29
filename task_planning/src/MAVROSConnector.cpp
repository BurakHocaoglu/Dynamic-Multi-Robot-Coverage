#include "task_planning/MAVROSConnector.h"

namespace task_planning {

    MAVROSConnector::MAVROSConnector() {
        //
    }

    MAVROSConnector::~MAVROSConnector() {
        //
    }

    void MAVROSConnector::initialize(ros::NodeHandle& n) override {
        node_handle_ = n;
        
        publishers_.insert(std::make_pair<std::string, ros::Publisher>("valocity_pub", node_handle_.advertise<>("")));
    }

    void MAVROSConnector::set_controller_mode(std::string&) override;
    void MAVROSConnector::set_controller_stream_rate(float) override;
    void MAVROSConnector::set_controller_rtl_altitude(float) override;
    void MAVROSConnector::set_controller_land_altitude(float) override;

    bool MAVROSConnector::arm(bool) override;
    bool MAVROSConnector::takeoff(float) override;
    bool MAVROSConnector::land() override;

    void MAVROSConnector::velocity_pub(float, float, float) override;
    void MAVROSConnector::position_pub(float, float, float, bool) override;

}