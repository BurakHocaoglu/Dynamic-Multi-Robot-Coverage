#ifndef COVERAGE_AGENT_H
#define COVERAGE_AGENT_H

#include "coverage_control/ControllerInterface.h"

namespace CCU = coverage_control_utils;

class Agent {

	public:
		Agent();
		Agent(uint8_t _id);
		~Agent();

		// Controller connection API
		bool Arm();
		bool Disarm();
		bool Takeoff(double altitude);
		bool Land(double altitude);
		bool Goto(Vector3d p);
		bool SetMode(std::string& mode);
		bool Setup();

		// Task spesific API
		void proximal_factor();
		void heading_factor();
		void obstacle_factor();
		void boundary_factor();
		void goal_factor();
		void compute_frontier();

		// Swarm API and rest
		void BroadcastState();
		void Move();
		void Log(std::string& msg);

		// Setters & Getters
		uint8_t get_id();
		std::string get_name();
		std::shared_ptr<ControllerInterface> get_controller_handle();
		coverage_control::FlightState get_flight_state();

		template<typename TValue>
		TValue get_uav_param(std::string& param_name);
		template<typename TValue>
		TValue get_task_param(std::string& param_name);

		template<typename TValue>
		bool set_uav_param(std::string& param_name, TValue value);
		template<typename TValue>
		bool set_task_param(std::string& param_name, TValue value);

	private:
		uint8_t id;
		std::string name;
		Vector3d goal;
		Vector3d position;
		Vector3d velocity;

		std::shared_ptr<ControllerInterface> controller_conn;
		mavros_msgs::State controller_state;
		coverage_control::FlightState flight_state;
		std::ostream logger;
		CCU::configs configuration;
		std::unordered_map<uint8_t, coverage_control::FlightState> neighbours;
};

#endif // COVERAGE_AGENT_H