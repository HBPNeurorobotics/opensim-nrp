#include <OpenSim/Simulation/osimSimulation.h>
#include <memory>

#include "MobilityDiscreteForce.h"

using SimTK::Vec3;
using namespace std;


void testSpinningSphere() // actually it is just sitting there at the moment
{
        // Setup OpenSim model
        OpenSim::Model osimModel;
        // We have to call buildSystem in order to obtain the multibody system for the integrator.
        osimModel.updBodySet().setMemoryOwner(false);
        osimModel.updForceSet().setMemoryOwner(false);
        osimModel.setGravity(Vec3(0));        
        
        auto osimSphere = std::make_unique<OpenSim::Body>("sphere", 1.0, Vec3(0), SimTK::Inertia::sphere(1.0));

        Vec3 locationInParent(0), orientationInParent(0), locationInBody(0), orientationInBody(0);
        
        /* This instance is made known to the system by handing a reference to itself to 
         * its body (not the parent body). This is done in the constructor. Later,
         * Body::connectToModel also invoces Joint::connectToModel.
         * Oddly, the joint is not owned by the body, nor does it appear to be owned by
         * the system.
         */
        auto blockToGround = std::make_unique<OpenSim::FreeJoint>("sphereToGround", 
            osimModel.getGroundBody(), locationInParent, orientationInParent, // specify parent and anchor point in its frame (?)
            *osimSphere, locationInBody, orientationInBody // specify child and the anchor point
        );

        osimModel.addBody(osimSphere.get());
                
        OpenSim::CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();
        jointCoordinateSet[0].setName("rot0");

        const OpenSim::CoordinateSet &modelCoords = osimModel.getCoordinateSet();
        for (int i=0; i<modelCoords.getSize(); ++i)
        {
          const OpenSim::Coordinate &c = modelCoords.get(i);
          std::cout << "coord " << i << " " << c.getName() << std::endl;
        }
        
        OpenSim::MobilityDiscreteForce force("rot0");
        osimModel.addForce(&force);
        
        osimModel.buildSystem();
        SimTK::State& osimState = osimModel.initializeState();
        
        OpenSim::Manager manager(osimModel);
        
        for (int iter = 0; iter < 15; ++iter)
        {
          osimModel.getMultibodySystem().realize(osimState, SimTK::Stage::Acceleration);
          std::cout << "t = " << iter << " Q    = " << osimState.getQ() << std::endl;
          std::cout << "t = " << iter << " U    = " << osimState.getU() << std::endl;
          std::cout << "t = " << iter << " Udot = " << osimState.getUDot() << std::endl;

          if (iter == 5)
            force.setForce(1.0);
          if (iter == 10)
            force.setForce(0.0);
            
          manager.setInitialTime(iter);
          manager.setFinalTime(iter+1);
          manager.integrate(osimState);
        }
        
        for (int idx = 0; idx < osimModel.getForceSet().getSize(); ++idx)
        {
          if (&osimModel.updForceSet()[idx] == &force)
            osimModel.updForceSet().remove(idx);
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