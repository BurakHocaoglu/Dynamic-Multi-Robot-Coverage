#include "coverage_control/ControllerInterface.h"

ControllerInterface::ControllerInterface() : type(ControllerType::NONE) {
	Initialize();
}

ControllerInterface::ControllerInterface(ControllerType t) : type(t) {
	Initialize();
}

ControllerType ControllerInterface::GetType() {
	return this->type;
}