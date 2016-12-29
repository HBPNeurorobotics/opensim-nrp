#ifndef OPENSIM_SIMULATION_H
#define OPENSIM_SIMULATION_H

#include <OpenSim/OpenSim.h>
#include "SimulationLogger.h"

class OpenSimSimulation
{   
    public:
        //OpenSimSimulation(const std::string& simulation_name, const std::string& scene_file);
        OpenSimSimulation(const std::string& simulation_name, const std::string& scene_file, const std::string& logDir="");
        ~OpenSimSimulation();
        
        void Init();
        void Fini();
        
        void Step();
        
        double GetTimeStep() const
        {
            return timeStep;   
        }
        
        void SetTimeStep(const double& step)
        {
            timeStep = step;
        }
        
        double GetSimTime() const
        {
            return currentTime;
        }

        void SetUseVisualization(bool val);

        bool GetUseVisualization() const
        {
          return useVisualization;
        }
        
    private: 
      std::string simulationName;
      std::string sceneFile;

      double currentTime;
      double timeStep;

      OpenSim::Model* osimModel;
      OpenSim::Manager* osimManager;
      OpenSim::ForceReporter* reporter;
      SimTK::RungeKuttaMersonIntegrator* integrator;
      SimTK::State si;

      SimTK::ContactTrackerSubsystem* contactTracker;

      OpenSim::JointReaction *jointReactionAnalysis;

      bool useVisualization;
	  
	  protected: 
		SimulationLogger* logger;
};

#endif //OPENSIM_SIMULATION_H
