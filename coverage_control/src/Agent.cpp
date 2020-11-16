#include "coverage_control/Agent.h"

Agent::Agent() {
	// ..
}

Agent::Agent(uint8_t _id) {
	// ..
}

Agent::~Agent() {
	// ..
}


// Controller connection API
bool Agent::Arm() {
	// ..
}

bool Agent::Disarm() {
	// ..
}

bool Agent::Takeoff(double altitude) {
	// ..
}

bool Agent::Land(double altitude) {
	// ..
}

bool Agent::Goto(Vector3d p) {
	// ..
}

bool Agent::SetMode(std::string& mode) {
	// ..
}

bool Agent::Setup() {
	// ..
}


// Task spesific API
void Agent::proximal_factor() {
	// ..
}

void Agent::heading_factor() {
	// ..
}

void Agent::obstacle_factor() {
	// ..
}

void Agent::boundary_factor() {
	// ..
}

void Agent::goal_factor() {
	// ..
}

void Agent::compute_frontier() {
	// ..
}


// Swarm API and rest
void Agent::BroadcastState() {
	// ..
}

void Agent::Move() {
	// ..
}

void Agent::Log(std::string& msg) {
	// ..
}


// Setters & Getters
uint8_t Agent::get_id() {
	// ..
}

std::string Agent::get_name() {
	// ..
}

std::shared_ptr<ControllerInterface> Agent::get_controller_handle() {
	// ..
}

coverage_control::FlightState Agent::get_flight_state() {
	// ..
}


template<typename TValue>
TValue Agent::get_uav_param(std::string& param_name) {
	// ..
}

template<typename TValue>
TValue Agent::get_task_param(std::string& param_name) {
	// ..
}


template<typename TValue>
bool Agent::set_uav_param(std::string& param_name, TValue value) {
	// ..
}

template<typename TValue>
bool Agent::set_task_param(std::string& param_name, TValue value) {
	// ..
}
