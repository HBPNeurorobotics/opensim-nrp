#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>

using SimTK::Vec3;
using namespace std;


void testBouncingSphere() // actually it is just sitting there at the moment
{
        double radius = 1.0;
        double start_height = 2.0;
        double mass = 1.0;
        double end_time = 1.0;
  
        // Setup OpenSim model
        OpenSim::Model osimModel;
        OpenSim::Body& osimGround = osimModel.getGroundBody();
        osimModel.setGravity(Vec3(0));
        
        OpenSim::Body *osimSphere = new OpenSim::Body("sphere", mass, Vec3(0), mass * SimTK::Inertia::sphere(radius));

        Vec3 locationInParent(0), orientationInParent(0), locationInBody(0), orientationInBody(0);
        OpenSim::FreeJoint *blockToGround = new OpenSim::FreeJoint("sphereToGround", osimGround, locationInParent, orientationInParent, *osimSphere, locationInBody, orientationInBody);
        
        OpenSim::CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();
        jointCoordinateSet[4].setDefaultValue(start_height); // set y-translation which is height
        
        osimModel.addBody(osimSphere);
        
        //osimModel->getMultibodySystem().realize(osim_state, Stage::Position );
        //osimModel.getMultibodySystem().realize(osimState, SimTK::Stage::Velocity);
                
        // Initialize the system and get the default state
        SimTK::State& osimState = osimModel.initSystem();
        
        std::cout << "initial state\n";
        std::cout << osimState.getQ() << std::endl;
        
        // Create the integrator for integrating system dynamics
        SimTK::RungeKuttaMersonIntegrator integrator(osimModel.getMultibodySystem());
        integrator.setAccuracy(1.0e-6);
        
        // Create the manager managing the forward integration and its outputs
        OpenSim::Manager manager(osimModel,  integrator);

        // Print out details of the model
        //osimModel.printDetailedInfo(osimState, cout);

        // Integrate from initial time to final time
        manager.setInitialTime(0.);
        manager.setFinalTime(2.);
        manager.integrate(osimState);
        
        std::cout << "end state\n";
        std::cout << osimState.getQ() << std::endl;
}


int main()
{
    try { testBouncingSphere(); }
    catch (const std::exception& e)
    {
                cout << e.what() <<endl;
    }
    return 0;
}