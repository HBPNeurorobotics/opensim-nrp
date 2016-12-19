#include "OpenSimSimulation.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

OpenSimSimulation::OpenSimSimulation(const std::string& simulation_name, const std::string& scene_file): currentTime(0.0), timeStep(0.0), simulationName(simulation_name), sceneFile(scene_file)
{
  osimModel = NULL;
  osimManager = NULL;
  reporter = NULL;
  integrator = NULL;
  si = NULL;
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
      osimModel->setUseVisualizer(false);

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

      reporter = new OpenSim::ForceReporter(osimModel);
      osimModel->addAnalysis(reporter);

      integrator = new SimTK::RungeKuttaMersonIntegrator(osimModel->getMultibodySystem());
      integrator->setAccuracy(1.0e-6);

      osimManager = new OpenSim::Manager(*osimModel);
    }
  }
}

void OpenSimSimulation::Fini()
{
  if (osimModel != NULL)
  {
    delete osimManager;
    osimManager = NULL;

    osimModel->removeAnalysis(reporter);
    delete reporter;
    reporter = NULL;

    if (integrator)
    {
      delete integrator;
      integrator = NULL;
    }

    delete osimModel;
    osimModel = NULL;
  }
}

void OpenSimSimulation::Step()
{
  std::cout << "OpenSimSimulation::Step(" << timeStep << ")" << std::endl;
  if (osimModel == NULL)
    return;

  std::cout << std::endl << "==========================================================================================" << std::endl;
  std::cout << "Integrate OpenSim from " << currentTime << " to " << (currentTime + timeStep) << std::endl;

  try
  {
    osimManager->setInitialTime(currentTime);
    osimManager->setFinalTime(osimManager->getInitialTime() + timeStep);

    SimTK::State& ws = osimModel->updWorkingState();
    
    //std::cout << "State: " << ws.toString() << std::endl;
    std::cout << "current Q (pos): " << ws.getQ() << std::endl;
    std::cout << "current U (vel): " << ws.getU() << std::endl;
    std::cout << "current Z (aux): " << ws.getZ() << std::endl;
    
    bool status = osimManager->doIntegration(ws, 1, timeStep);
    if (status)
    {
      const SimTK::State& istate =osimManager->getIntegrator().getState();

      //std::cout << " Working state system stage: " << ws.getSystemStage().getName() << std::endl;
      //std::cout << " Integrator state system stage: " << istate.getSystemStage().getName() << std::endl;

      currentTime += timeStep;

      /*if (this->openSimPub && !transport::getMinimalComms())
        {
          msgs::OpenSimMuscles muscles_msg;

          if (osimModel->getBodySet().contains("block"))
          {
            std::cout << "Found Gazebo visual to synchronize: block" << std::endl;
            const OpenSim::Body& blockBody = osimModel->getBodySet().get("block");
            const SimTK::MobilizedBodyIndex& bodyIndex = blockBody.getIndex();
            const SimTK::MobilizedBody& simTkBody = osimModel->getMultibodySystem().getMatterSubsystem().getMobilizedBody(bodyIndex);
            const SimTK::Transform& bodyTransform = simTkBody.getBodyTransform(istate);

            muscles_msg.set_osobj_name(blockBody.getName());
            muscles_msg.mutable_osobj_position()->set_x(bodyTransform.p()[0]);
            muscles_msg.mutable_osobj_position()->set_y(bodyTransform.p()[1]);
            muscles_msg.mutable_osobj_position()->set_z(bodyTransform.p()[2]);
          }

          const OpenSim::Set<OpenSim::Muscle>& muscles = osimModel->getMuscles();
          for (int k = 0; k < muscles.getSize(); ++k)
          {
            msgs::OpenSimMuscle* muscle_msg = muscles_msg.add_muscle();
            OpenSim::Muscle& muscle = muscles[k];
            double mfl = muscle.getFiberLength(istate);
            double mfv = muscle.getFiberVelocity(istate);
            double mff = muscle.getFiberForce(istate);

            //double mfl = muscle.getTendonLength(istate);
            //double mfv = muscle.getTendonVelocity(istate);
            //double mff = muscle.getTendonForce(istate);

            muscle_msg->set_name(muscle.getName());
            muscle_msg->set_fiberlength(mfl);
            muscle_msg->set_fibervelocity(mfv);
            muscle_msg->set_fiberforce(mff);

            OpenSim::Array<OpenSim::PathPoint*> current_path = muscle.getGeometryPath().getCurrentPath(istate);
            const OpenSim::PathPointSet& muscle_path = muscle.getGeometryPath().getPathPointSet();

            if (muscle_path.getSize() == 2 && current_path.getSize() == 2)
            {

              OpenSim::PathPoint* pt_1 = current_path.get(0);
              OpenSim::PathPoint* pt_2 = current_path.get(1);

              muscle_msg->mutable_attachment_1()->set_x(pt_1->getLocation()[0]);
              muscle_msg->mutable_attachment_1()->set_y(pt_1->getLocation()[1]);
              muscle_msg->mutable_attachment_1()->set_z(pt_1->getLocation()[2]);

              muscle_msg->mutable_attachment_2()->set_x(pt_2->getLocation()[0]);
              muscle_msg->mutable_attachment_2()->set_y(pt_2->getLocation()[1]);
              muscle_msg->mutable_attachment_2()->set_z(pt_2->getLocation()[2]);
              std::cout << " - muscle " << k << ": attachment_1 = " << pt_1->getLocation()[0] << "," << pt_1->getLocation()[1] << "," << pt_1->getLocation()[2]
                        << ", attachment_2 = " << pt_2->getLocation()[0] << "," << pt_2->getLocation()[1] << "," << pt_2->getLocation()[2]
                        << ", fiber length = " << mfl << ", fiber velocity = " << mfv << ", fiber force = " << mff << std::endl;


            }
          }

          msgs::Set(muscles_msg.mutable_time(), simTime);
          this->openSimPub->Publish(muscles_msg);
        }*/
    }
    else
    {
      std::cout << "Warning: doIntegration call failed." << std::endl;
    }
  }
  catch (OpenSim::Exception& ex)
  {
    std::cerr << "OpenSim exception: " << ex.getMessage() << std::endl;
  }
}
