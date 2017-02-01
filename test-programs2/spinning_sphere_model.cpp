#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>
#include <memory>

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
        
        std::unique_ptr<OpenSim::Body> osimSphere = std::make_unique<OpenSim::Body>("sphere", mass, Vec3(0), mass * SimTK::Inertia::sphere(radius));

        Vec3 locationInParent(0), orientationInParent(0), locationInBody(0), orientationInBody(0);
        
        OpenSim::FreeJoint blockToGround("sphereToGround", 
            osimGround, locationInParent, orientationInParent, // specify parent and anchor point in its frame (?)
            *osimSphere, locationInBody, orientationInBody // specify child and the anchor point
        );
        
        // What each value means depends on the type of joint ?????
        // For FreeJoint we have 3 angles and after that 3 position coordinates.
        OpenSim::CoordinateSet& jointCoordinateSet = blockToGround.upd_CoordinateSet();
        jointCoordinateSet[4].setDefaultValue(start_height); // set y-translation which is height
        jointCoordinateSet[0].setDefaultSpeedValue(3.14 * 2.);  // 2 pi per sec ?!
        
        // Model takes ownership!        
        osimModel.addBody(osimSphere.release());
        
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

        double t = 0;
        int Nsteps = 10;
        double dt = (end_time - t) / Nsteps;
        int i = 0;
        while (true)
        {
          std::cout << "t = " << t << " state = " << osimState.getQ() << std::endl;

          if (++i > Nsteps) break;
          
          // Does the right thing! Starts at t and steps forward until t+dt.
          manager.setInitialTime(t);
          manager.setFinalTime(t+dt);
          manager.integrate(osimState);
          t += dt;
        }
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