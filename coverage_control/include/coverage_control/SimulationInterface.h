#ifndef SIMULATOR_INTERFACE_H
#define SIMULATOR_INTERFACE_H

#include "coverage_control/Agent.h"

class SimulatorInterface {
	public:
		SimulatorInterface();
		~SimulatorInterface();

		bool AddAgent(Agent& a);
		void Run();

		SimulatorType GetType();
		// ...

	private:
		void Initialize();
		void Load();

		SimulatorType type;
};

#endif // SIMULATOR_INTERFACE_H