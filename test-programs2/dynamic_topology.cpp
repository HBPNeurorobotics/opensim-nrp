#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>
#include <memory>

using SimTK::Vec3;
using namespace std;

std::unique_ptr<SimTK::Integrator> IntegratorFactory(const SimTK::MultibodySystem &system)
{
  // Integrator uses pimpl ideom, but has no move nor assignment operator, so we are better off handling it through a pointer.
  auto integ = std::make_unique<SimTK::RungeKuttaMersonIntegrator>(system);
  integ->setAccuracy(1.0e-6);
  return integ;
}


/* Equivalent to Simbody's mobilized bodies. For testing purposes 
   we use hardcoded FreeJoints.  */
struct DemoMob 
{
  OpenSim::Body body;
  OpenSim::FreeJoint joint;
  
  DemoMob(const DemoMob &) = delete;
  const DemoMob& operator=(const DemoMob &) = delete;
  
  DemoMob(const std::string &name, OpenSim::Body &parent)
    : body(name, 1.0, Vec3(0), SimTK::Inertia::sphere(1.0)),
      joint(name+"to"+parent.getName(), parent, Vec3(0), Vec3(0), body, Vec3(0), Vec3(0))
  {
    // This is a lot of work to be done in the initializer above. 
    // Unfortunately, Body nor Joint provide move assignment with which we could have initialized them here.
  }
};


/* Purpose of this function is to explore how we can obtain the pose of bodies,
 * and how to add bodies to the simulation in between time steps. The difficulty
 * in the later case lies therein that the internal Simbody system is rebuild when
 * the system topology changes. Therefore we need a temporary backup of the state 
 * variables of the existing bodies which is then used to initialize the new state. */
void testDynamicTopology()
{
        const double start_height = 2.0;
        const double dt = 0.1;
  
        OpenSim::Model osimModel;
        OpenSim::Body& osimGround = osimModel.getGroundBody();
        osimModel.setGravity(Vec3(0));        
        osimModel.updBodySet().setMemoryOwner(false);
        OpenSim::Manager manager(osimModel);

        /* Some "local functions" for convenience */
        
        double t = 0;
        auto do_step = [&]()
        {
          manager.setInitialTime(t);
          manager.setFinalTime(t+dt);
          manager.integrate(osimModel.updWorkingState());
          t += dt;
        };
        
        auto update_system = [&]()
        {
          /* Have to rebuild the system. Otherwise the internal Simbody representation
          * will know nothing of our new body. State variables of first body are initialized
          * from default values set above. */
          osimModel.buildSystem();
          auto integrator = IntegratorFactory(osimModel.getMultibodySystem());
          // Setting a new integrator does not free the old one, so we have to do it manually.
          delete &manager.getIntegrator();
          manager.setIntegrator(integrator.release());
          osimModel.initializeState();
        };
        
        
        /* First body. And set initial conditions. */
        auto mob1 = std::make_unique<DemoMob>("mob1", osimGround);
        { OpenSim::CoordinateSet& jointCoordinateSet = mob1->joint.upd_CoordinateSet();
        jointCoordinateSet[4].setDefaultValue(start_height); // set y-translation which is height
        jointCoordinateSet[0].setDefaultSpeedValue(3.14 * 2.);  // 2 pi per sec ?!
        }
        osimModel.addBody(&mob1->body);
        
        update_system();
        
        std::cout << "Initial state\n";
        std::cout << osimModel.getWorkingState().getQ() << std::endl;

        /* Do time steps */
        for (int i=0; i<5; ++i)
        {
          auto &osimState = osimModel.updWorkingState();
          osimModel.getMultibodySystem().realize(osimState, SimTK::Stage::Velocity);

          std::cout << "t = " << t << " state = " << osimState.getQ() << std::endl;
          std::cout << "mob1 pose = " << osimModel.getSimbodyEngine().getTransform(osimState, mob1->body) << std::endl;
          
          do_step();
        }
  
        /* Add second body. */
        auto mob2 = std::make_unique<DemoMob>("mob2", osimGround);
        { OpenSim::CoordinateSet& jointCoordinateSet = mob2->joint.upd_CoordinateSet();
        jointCoordinateSet[4].setDefaultValue(-start_height); // set y-translation which is height
        jointCoordinateSet[0].setDefaultSpeedValue(3.14 * 2.);  // 2 pi per sec ?!
        }
        osimModel.addBody(&mob2->body);

        /* OpenSim provides persistent storage of initial states in the Coordinate class 
         * which we divert slightly from its intended use. */
        { 
          auto &osimState = osimModel.updWorkingState();
          OpenSim::CoordinateSet& jc = mob1->joint.upd_CoordinateSet();
          for (int i=0; i<mob1->joint.numCoordinates(); ++i)
          {
            jc[i].setDefaultValue(jc[i].getValue(osimState));
            jc[i].setDefaultSpeedValue(jc[i].getSpeedValue(osimState)); 
          }
        }

        update_system();
        
        /* Do more time steps */
        for (int i=0; i<5; ++i)
        {
          auto &osimState = osimModel.updWorkingState();
          osimModel.getMultibodySystem().realize(osimState, SimTK::Stage::Velocity);

          std::cout << "t = " << t << " state = " << osimState.getQ() << std::endl;
          std::cout << "mob1 pose = " << osimModel.getSimbodyEngine().getTransform(osimState, mob1->body) << std::endl;
          std::cout << "mob2 pose = " << osimModel.getSimbodyEngine().getTransform(osimState, mob2->body) << std::endl;
          do_step();
        }
        
        /* Lets try to remove the first body */
        osimModel.updBodySet().remove(&mob1->body);
        mob1 = nullptr; // delete
        
        { 
          auto &osimState = osimModel.updWorkingState();
          OpenSim::CoordinateSet& jc = mob2->joint.upd_CoordinateSet();
          for (int i=0; i<mob2->joint.numCoordinates(); ++i)
          {
            jc[i].setDefaultValue(jc[i].getValue(osimState));
            jc[i].setDefaultSpeedValue(jc[i].getSpeedValue(osimState)); 
          }
        }

        update_system();
        
        /* Take some final time steps */
        for (int i=0; i<5; ++i)
        {
          auto &osimState = osimModel.updWorkingState();
          osimModel.getMultibodySystem().realize(osimState, SimTK::Stage::Velocity);

          std::cout << "t = " << t << " state = " << osimState.getQ() << std::endl;
          std::cout << "mob2 pose = " << osimModel.getSimbodyEngine().getTransform(osimState, mob2->body) << std::endl;
          do_step();
        }
}


int main()
{
    try { testDynamicTopology(); }
    catch (const std::exception& e)
    {
                cout << e.what() <<endl;
    }
    return 0;
}