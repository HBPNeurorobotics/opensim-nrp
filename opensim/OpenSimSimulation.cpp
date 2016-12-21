#include "OpenSimSimulation.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

OpenSimSimulation::OpenSimSimulation(const std::string& simulation_name, const std::string& scene_file): currentTime(0.0), timeStep(0.0), simulationName(simulation_name), sceneFile(scene_file), useVisualization(false)
{
  osimModel = NULL;
  osimManager = NULL;
  reporter = NULL;
  integrator = NULL;

  ///
  jointReactionAnalysis = NULL;
}

OpenSimSimulation::~OpenSimSimulation()
{

}


void OpenSimSimulation::Init()
{
  std::cout << "OpenSimSimulation::Init() -- sceneFile: " << sceneFile << std::endl;
  if (!sceneFile.empty())
  {
    boost::filesystem::path sceneFilePath = boost::filesystem::canonical(sceneFile);
    std::cout << " check for scene file: " << sceneFilePath.string() << std::endl;
    if (boost::filesystem::exists(sceneFilePath.string()))
    {
      std::string simulationFilePath = sceneFilePath.string();
      osimModel = new OpenSim::Model(simulationFilePath);
      std::cout << "Loaded model: " << osimModel->getName() << std::endl;

      std::cout << " number of bodies: " << osimModel->getNumBodies() << ", number of contact geometries: " << osimModel->getNumContactGeometries() << std::endl;
      std::cout << " number of joints: " << osimModel->getNumJoints() << std::endl;

      //osimModel->setUseVisualizer(true);
      osimModel->setUseVisualizer(this->useVisualization);

      // Initialize the system and get the default state
      SimTK::State& isi = osimModel->initSystem();
      osimModel->getMultibodySystem().realize(isi, SimTK::Stage::Velocity);

      std::cout << " muscles in model: " << osimModel->getMuscles().getSize() << std::endl;
      const OpenSim::Set<OpenSim::Muscle>& muscleSet = osimModel->getMuscles();
      for (int k = 0; k < muscleSet.getSize(); ++k)
      {
        std::cout << " * " << muscleSet.get(k).getName() << std::endl;
        muscleSet[k].setActivation(isi, 0.01);
      }

      osimModel->equilibrateMuscles(isi);

      this->si = osimModel->initializeState();

      contactTracker = new SimTK::ContactTrackerSubsystem(osimModel->updMultibodySystem());

      reporter = new OpenSim::ForceReporter(osimModel);
      osimModel->addAnalysis(reporter);

      integrator = new SimTK::RungeKuttaMersonIntegrator(osimModel->getMultibodySystem());
      integrator->setAccuracy(1.0e-6);

      osimManager = new OpenSim::Manager(*osimModel);

      //////////////
      const OpenSim::JointSet& joints = osimModel->getJointSet();
      int numJoints = osimModel->getNumJoints();

      OpenSim::Array< std::string > jointNames;
      
      std::cout << "joint ranges: " << std::endl;
      for (int u = 0; u < numJoints; u++)
	{
	  OpenSim::Joint *joint = &(joints[u]);	  
	  jointNames.append(joint->getName());
	    
	  OpenSim::CoordinateSet jointCoordinates = joint->getCoordinateSet();
	  OpenSim::Coordinate coordinate = jointCoordinates[0];
	  std::cout << coordinate.getRangeMin() << ", " << coordinate.getRangeMax()  << " --- " ;
	}
      std::cout << std::endl;

      ////
      const OpenSim::ConstraintSet& constraints = osimModel->getConstraintSet();

      std::cout << "constraints: " << std::endl;
      for (int u = 0; u < constraints.getSize(); u++)
	{
	  OpenSim::Constraint *constraint = &(constraints[u]);
	  std::cout << constraint->getName() << "--- " ;
	}
      std::cout << std::endl;

      //// 
      const OpenSim::ForceSet& forces = osimModel->getForceSet();

      std::cout << "forces with record labels: " << std::endl;
      for (int u = 0; u < forces.getSize(); u++)
      {
        OpenSim::Force *force = &(forces[u]);
        std::cout << force->getName() << ": " << force->getRecordLabels() << std::endl;
      }
      std::cout << std::endl;

      ////
      jointReactionAnalysis = new OpenSim::JointReaction();
      //jointReactionAnalysis->setJointNames(jointNames);
      jointReactionAnalysis->setModel(*osimModel);
      osimModel->addAnalysis(jointReactionAnalysis);

      //jointReactionAnalysis->begin(isi);
    }
  }
}

void OpenSimSimulation::Fini()
{
  if (osimModel != NULL)
  {
    delete osimManager;
    osimManager = NULL;

    delete contactTracker;
    contactTracker = NULL;

    reporter->print("test.out");

    osimModel->removeAnalysis(reporter);
    delete reporter;
    reporter = NULL;

    if (integrator)
    {
      delete integrator;
      integrator = NULL;
    }

    ////
    if (jointReactionAnalysis)
    {
      /*SimTK::State asd = osimModel->getWorkingState();
      
	jointReactionAnalysis->end(asd);*/
      jointReactionAnalysis->printResults("jointReactionResults","/tmp/");
      
      osimModel->removeAnalysis(jointReactionAnalysis);
    
      delete jointReactionAnalysis;
      jointReactionAnalysis = NULL;
    }
    ////

    delete osimModel;
    osimModel = NULL;
  }
}

void OpenSimSimulation::Step()
{
  //std::cout << "OpenSimSimulation::Step(" << timeStep << ")" << std::endl;
  if (osimModel == NULL)
    return;

  std::cout << std::endl << "==========================================================================================" << std::endl;
  std::cout << "Integrate OpenSim from " << currentTime << " to " << (currentTime + timeStep) << std::endl;

  try
  {
    osimManager->setInitialTime(currentTime);
    osimManager->setFinalTime(osimManager->getInitialTime() + timeStep);

    if (!osimModel->isValidSystem())
    {
      std::cout << "Call initializeState..." << std::endl;
      SimTK::State& initial_state = osimModel->initializeState();
      std::cout << "Result: " << initial_state << std::endl;
    }
    SimTK::State& ws = osimModel->updWorkingState();
    
    //std::cout << "State: " << ws.toString() << std::endl;
    std::cout << "current Q (pos): " << ws.getQ() << std::endl;
    std::cout << "current U (vel): " << ws.getU() << std::endl;
    std::cout << "current Z (aux): " << ws.getZ() << std::endl;
    

    bool status = osimManager->doIntegration(ws, 1, timeStep);
    if (status)
    {
      const SimTK::State& istate = osimManager->getIntegrator().getState();

      std::cout << "Stage after integration: " << istate.getSystemStage().getName() << std::endl;

      std::cout << "new Q (pos): " << istate.getQ() << std::endl;
      std::cout << "new U (vel): " << istate.getU() << std::endl;
      std::cout << "new Z (aux): " << istate.getZ() << std::endl;

      /*int jt_ct = osimModel->getMatterSubsystem().getNumConstraints();
      for (int k = 0; k < jt_ct; ++k)
      {
        osimModel->getMatterSubsystem().
      }*/

      for (int k = 0; k < osimModel->getNumJoints(); ++k)
      {
        const OpenSim::Joint& jt = osimModel->getJointSet().get(k);
        /*const SimTK::ConstraintIndex ci(k);
        const SimTK::Constraint& ct = osimModel->getMatterSubsystem().getConstraint(ci);

        std::cout << " - Joint " << k << " = " << jt.getName() << std::endl;

        const SimTK::Constraint::Ball* bj = dynamic_cast<const SimTK::Constraint::Ball*>(&ct);
        if (bj)
          std::cout << "   SimBody: Ball joint." << std::endl;

        const SimTK::Constraint::Rod* rj = dynamic_cast<const SimTK::Constraint::Rod*>(&ct);
        if (rj)
          std::cout << "   SimBody: Rod joint." << std::endl;

        const SimTK::Constraint::Weld* wj = dynamic_cast<const SimTK::Constraint::Weld*>(&ct);
        if (wj)
          std::cout << "   SimBody: Weld joint." << std::endl;*/

        const OpenSim::CoordinateSet& ocs = jt.getCoordinateSet();
        for (int n = 0; n < ocs.getSize(); ++n)
        {
          const OpenSim::Coordinate& coord = ocs.get(n);

          const SimTK::ConstraintIndex& lci = coord.getLockedConstraintIndex();
          const SimTK::ConstraintIndex& cci = coord.getClampedConstraintIndex();
          const SimTK::ConstraintIndex& pci = coord.getPrescribedConstraintIndex();

          osimModel->getMatterSubsystem().getConstraint(lci);
          osimModel->getMatterSubsystem().getConstraint(cci);
          osimModel->getMatterSubsystem().getConstraint(pci);
        }
      }
      //jointReactionAnalysis->step(istate,1);

      std::cout << "new Q (pos): " << istate.getQ() << std::endl;
      std::cout << "new U (vel): " << istate.getU() << std::endl;
      std::cout << "new Z (aux): " << istate.getZ() << std::endl;

      const SimTK::GeneralContactSubsystem& contactSystem = osimModel->getMultibodySystem().getContactSubsystem();
      int contactSetsCount = contactSystem.getNumContactSets();

      std::cout << "=== contacts: " << contactSetsCount << " ===" << std::endl;
      for (int k = 0; k < contactSetsCount; ++k)
      {
        SimTK::ContactSetIndex csi(k);
        const SimTK::Array_<SimTK::Contact>& contacts = contactSystem.getContacts(istate, csi);

        std::cout << " - Set " << k << ": " << contacts.size() << " entries." << std::endl;
        for (int l = 0; l < contacts.size(); ++l)
        {
          SimTK::Contact contact = contacts.at(l);
          std::cout << "   - Set " << k << ", entry " << l << ": " << contact << std::endl;

          if (SimTK::PointContact::isInstance(contact))
          {
            std::cout << "     -> PointContact." << std::endl;
            const SimTK::PointContact& c = static_cast<const SimTK::PointContact&>(contact);
            std::cout << "        depth = " << c.getDepth() << ", normal = " << c.getNormal() << std::endl;
          }
          else if (SimTK::CircularPointContact::isInstance(contact))
          {
            std::cout << "     -> CircularPointContact." << std::endl;
            const SimTK::CircularPointContact& c = static_cast<const SimTK::CircularPointContact&>(contact);
            std::cout << "        " << c << std::endl;
          }
          else if (SimTK::EllipticalPointContact::isInstance(contact))
          {
            std::cout << "     -> EllipticalPointContact." << std::endl;
            const SimTK::EllipticalPointContact& c = static_cast<const SimTK::EllipticalPointContact&>(contact);
            std::cout << "        " << c << std::endl;
          }
        }
      }

      /*int jt_ct = osimModel->getMatterSubsystem().getNumConstraints();
      for (int k = 0; k < jt_ct; ++k)
      {

      }*/

      std::cout << "=== Joints: " << osimModel->getNumJoints() << " ===" << std::endl;
      for (int k = 0; k < osimModel->getNumJoints(); ++k)
      {
        const OpenSim::Joint& jt = osimModel->getJointSet().get(k);
        const SimTK::ConstraintIndex ci(k);
        const SimTK::Constraint& ct = osimModel->getMatterSubsystem().getConstraint(ci);

        osimModel->getMultibodySystem().realize(istate, SimTK::Stage::HighestRuntime);

        std::cout << " - Joint " << k << " = " << jt.getName() << std::endl;

        /*if (ct.isDisabled(istate))
        {
          SimTK::State tmp(istate);
          ct.enable(tmp);

          SimTK::Vector_<SimTK::SpatialVec> ct_forces = ct.getConstrainedBodyForcesAsVector(istate);
          std::cout << ct_forces << std::endl;
        }
        try
        {
          if (istate.getSystemStage() == SimTK::Stage::Dynamics)
          {
            SimTK::State tmp(istate);
            if (tmp.getSystemStage() == SimTK::Stage::Dynamics)
            {
              ct.enable(tmp);
              if (!ct.isDisabled(tmp))
              {
                std::cout << ct.getConstrainedMobilityForcesAsVector(tmp) << std::endl;
                std::cout << ct.getConstrainedBodyForcesAsVector(tmp) << std::endl;
              }
            }
          }
        }
        catch (SimTK::Exception::ErrorCheck& ex)
        {

        }*/
      }

      //std::cout << " Working state system stage: " << ws.getSystemStage().getName() << std::endl;
      //std::cout << " Integrator state system stage: " << istate.getSystemStage().getName() << std::endl;

      currentTime += timeStep;

      //// 
      const OpenSim::ForceSet& forces = osimModel->getForceSet();

      std::cout << "=== Forces: " << forces.getSize() << " ===" << std::endl;
      for (int u = 0; u < forces.getSize(); u++)
      {
        OpenSim::Force *force = &(forces[u]);


        std::cout << " * " << force->getName() << " of class " << force->getClassName() << ": " << std::endl;

        OpenSim::Array<double> fValues = force->getRecordValues(istate);
        OpenSim::Array<std::string> fLabels = force->getRecordLabels();

        for (int m = 0; m < fLabels.size(); ++m)
          std::cout << fLabels[m] << " = " << fValues[m] << ";";

        std::cout << std::endl;
        SimTK::ForceIndex force_idx = force->getForceIndex();
        const SimTK::Force& simTkForce = osimModel->getForceSubsystem().getForce(force_idx);
        if (SimTK::HuntCrossleyForce::isInstanceOf(simTkForce))
        {
          const SimTK::HuntCrossleyForce* hkf = (const SimTK::HuntCrossleyForce*)(&simTkForce);
          std::cout << "  HuntCrossleyForce: " << hkf->getTransitionVelocity() << std::endl;
        }

        if (SimTK::ElasticFoundationForce::isInstanceOf(simTkForce))
        {
          const SimTK::ElasticFoundationForce* eff = (const SimTK::ElasticFoundationForce*)(&simTkForce);
          std::cout << "  ElasticFoundationForce: " << eff->getTransitionVelocity() << std::endl;
        }
      }
      std::cout << std::endl;

      const OpenSim::ComponentSet& mcs = osimModel->getMiscModelComponentSet();
      std::cout << "=== Model components: " << mcs.getSize() << " ===" << std::endl;
      for (int k = 0; k < mcs.getSize(); ++k)
      {
        OpenSim::ModelComponent& mk = mcs[k];
        std::cout << " -> " << mk.getName() << mk.getClassName() << std::endl;
      }

      if (istate.getNumSubsystems() > 0)
      {
        // get contact snapshot
        SimTK::Real dummy;
        bool active_ct_state = contactTracker->realizeActiveContacts(istate, false, dummy);
        if (active_ct_state)
          std::cout << "Tracker updated active contacts." << std::endl;
        else
          std::cout << "Tracker failed updating active contacts!" << std::endl;

        bool pred_ct_state = contactTracker->realizePredictedContacts(istate, false, dummy);
        if (pred_ct_state)
          std::cout << "Tracker updated predicted contacts." << std::endl;
        else
          std::cout << "Tracker failed updating predicted contacts!" << std::endl;

        const SimTK::ContactSnapshot &contactSnapshot =
          this->contactTracker->getActiveContacts(istate);

        int numc = contactSnapshot.getNumContacts();
        std::cout << "=== Contact count in contactTracker: " << numc << " ===" << std::endl;
      }

      // std::cout << "State dump istate: " << std::endl << istate << std::endl;
    }
    else
    {
      std::cout << "Warning: doIntegration call failed." << std::endl;
    }
  }
  catch (OpenSim::Exception& ex)
  {
    std::cerr << "OpenSim exception: " << ex.getMessage() << std::endl;
    std::cerr << ex.what() << std::endl;
  }
}
