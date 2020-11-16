#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "coverage_control/Utils.h"

class ControllerInterface {

	public:
		ControllerInterface();
		ControllerInterface(ControllerType t);
		~ControllerInterface();

		virtual bool Arm();
		virtual bool Disarm();
		virtual bool Takeoff(const double altitude);
		virtual bool Land();
		virtual bool Goto();
		virtual bool SetMode(std::string mode);

		virtual void Initialize();
		virtual void Load();

		ControllerType GetType();

	private:
		ControllerType type;
		ros::Publisher goal_pub;
		ros::ServiceClient arm_client;
		ros::ServiceClient takeoff_client;
		ros::ServiceClient land_client;
		ros::ServiceClient mode_client;
		ros::ServiceClient param_set_client;
		ros::ServiceClient param_get_client;
};

#endif // CONTROLLER_INTERFACE_H