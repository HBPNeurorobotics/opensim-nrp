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
		
		class ElasticFoundationForce_Parameters {
			public:
				SimTK::Array_<SimTK::Vec3> springPosition;
				SimTK::Array_<SimTK::UnitVec3> springNormal;
				SimTK::Array_<SimTK::Real> springArea;
		};
	
		void calcContactInfo(const SimTK::State& state, 
			SimTK::ContactSurfaceIndex meshIndex, SimTK::ContactSurfaceIndex otherBodyIndex, 
			const ElasticFoundationForce_Parameters& param, const std::set<int>& insideFaces,
			/*Real areaScale, Vector_<SpatialVec>& bodyForces, Real& pe,*/
			SimTK::GeneralContactSubsystem subsystem,
			SimTK::ContactSetIndex set) const;

		std::map<SimTK::ContactSurfaceIndex, OpenSimSimulation::ElasticFoundationForce_Parameters>  parameters;
};


#endif //OPENSIM_SIMULATION_H
