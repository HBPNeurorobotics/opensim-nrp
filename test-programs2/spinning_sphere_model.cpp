#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>
#include <memory>

using SimTK::Vec3;
using namespace std;

#define MODEL_OWNS_BODIES 0

std::unique_ptr<SimTK::Integrator> IntegratorFactory(const SimTK::MultibodySystem &system)
{
  // Integrator uses pimpl ideom, but has no move nor assignment operator, so we are better off handling it through a pointer.
  auto integ = std::make_unique<SimTK::RungeKuttaMersonIntegrator>(system);
  integ->setAccuracy(1.0e-6);
  return integ;
}


void testSpinningSphere() // actually it is just sitting there at the moment
{
        double radius = 1.0;
        double start_height = 2.0;
        double mass = 1.0;
        double end_time = 1.0;
  
        // Setup OpenSim model
        OpenSim::Model osimModel;
        // We have to call buildSystem in order to obtain the multibody system for the integrator.
        osimModel.buildSystem();
        auto integrator = IntegratorFactory(osimModel.getMultibodySystem());
        
#if (!MODEL_OWNS_BODIES)
        osimModel.updBodySet().setMemoryOwner(false);
#endif
        OpenSim::Body& osimGround = osimModel.getGroundBody();
        osimModel.setGravity(Vec3(0));        
        
        auto osimSphere = std::make_unique<OpenSim::Body>("sphere", mass, Vec3(0), mass * SimTK::Inertia::sphere(radius));

        Vec3 locationInParent(0), orientationInParent(0), locationInBody(0), orientationInBody(0);
        
        /* This instance is made known to the system by handing a reference to itself to 
         * its body (not the parent body). This is done in the constructor. Later,
         * Body::connectToModel also invoces Joint::connectToModel.
         * Oddly, the joint is not owned by the body, nor does it appear to be owned by
         * the system.
         */
        auto blockToGround = std::make_unique<OpenSim::FreeJoint>("sphereToGround", 
            osimGround, locationInParent, orientationInParent, // specify parent and anchor point in its frame (?)
            *osimSphere, locationInBody, orientationInBody // specify child and the anchor point
        );
        
        // What each value means depends on the type of joint ?????
        // For FreeJoint we have 3 angles and after that 3 position coordinates.
        OpenSim::CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();
        jointCoordinateSet[4].setDefaultValue(start_height); // set y-translation which is height
        jointCoordinateSet[0].setDefaultSpeedValue(3.14 * 2.);  // 2 pi per sec ?!
        
        // Model takes ownership by default!        
        // However, we could use osimModel.updBodySet().setMemoryOwner(false);
        // to keep owenership of the body, and all bodies and possibly everything added to the model.
#if MODEL_OWNS_BODIES
        osimModel.addBody(osimSphere.release());
#else
        osimModel.addBody(osimSphere.get());
#endif
        // Since the topology changed (new object added), we have to rebuild the internal data structures.
        osimModel.buildSystem();
        // Internal pointer to the MultibodySystem got invalidated since the old system got deleted. Thus we have to get a new integrator, as well.
        integrator = IntegratorFactory(osimModel.getMultibodySystem());

        // Having done that, we can initialize and obtain our state vector.
        // Alternatively, initSystem can be called which invokes buildSystem and initializeState.
        SimTK::State& osimState = osimModel.initializeState();
                
        // Regarding the life time of the joint: Deletion of blockToGround 
        // at this point results in a segfault. Therefore, I think that 
        // OpenSim::FreeJoint and friends must stay alive during the lifetime
        // of the system.
        /* blockToGround = nullptr; */ // BOOM!
        
        std::cout << "initial state\n";
        std::cout << osimState.getQ() << std::endl;
        
        // Create the manager managing the forward integration and its outputs
        OpenSim::Manager manager(osimModel,  *integrator);

        // Print out details of the model
        //osimModel.printDetailedInfo(osimState, cout);

        double t = 0;
        int Nsteps = 10;
        double dt = (end_time - t) / Nsteps;
        int i = 0;
        while (true)
        {
          std::cout << "t = " << t << " state = " << osimState.getQ() << std::endl;
#if (!MODEL_OWNS_BODIES)
          osimModel.getMultibodySystem().realize(osimState, SimTK::Stage::Velocity);
          SimTK::Transform pose = osimModel.getSimbodyEngine().getTransform(osimState, *osimSphere);
          std::cout << "sphere pose = " << pose << std::endl;
#endif
          if (++i > Nsteps) break;
          
          // Does the right thing! Starts at t and steps forward until t+dt.
          // However, integrate does lots of wired sh**. Its probably going 
          // to be really slow.
          manager.setInitialTime(t);
          manager.setFinalTime(t+dt);
          manager.integrate(osimState);
          t += dt;
        }
}


int main()
{
    try { testSpinningSphere(); }
    catch (const std::exception& e)
    {
                cout << e.what() <<endl;
    }
    return 0;
}