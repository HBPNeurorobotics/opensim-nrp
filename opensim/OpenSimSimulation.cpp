#include "OpenSimSimulation.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>

#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/ComponentSet.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

#include <simbody/internal/Constraint.h>
#include <simbody/internal/Constraint_Ball.h>
#include <simbody/internal/Constraint_Rod.h>
#include <simbody/internal/Constraint_Weld.h>

// tst Anfang
#include <SimTKcommon/internal/ExceptionMacros.h>
// tst Ende

#include <string>

OpenSimSimulation::OpenSimSimulation(const std::string& simulation_name, const std::string& scene_file, const std::string& logDir/* ="" */): 
	  currentTime(0.0)
	, timeStep(0.0)
	, simulationName(simulation_name)
	, sceneFile(scene_file)
	, useVisualization(false)
{
  osimModel = NULL;
  osimManager = NULL;
  reporter = NULL;
  integrator = NULL;

  ///
  jointReactionAnalysis = NULL;
  
  if (!logDir.empty())
  {
	logger = new SimulationLogger(logDir);
  }
}

OpenSimSimulation::~OpenSimSimulation()
{
	if (logger)
	{
		delete logger;
	}
}

void OpenSimSimulation::SetUseVisualization(bool val)
{
  if (val != useVisualization)
  {
    osimModel->setUseVisualizer(this->useVisualization);
    useVisualization = val;
  }
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
	  
	  std::cout << "-- initialized contact tracker subsystem - this tracker subsystem manages " << contactTracker->getNumSurfaces() << " surfaces --" << std::endl;

      reporter = new OpenSim::ForceReporter(osimModel);
      osimModel->addAnalysis(reporter);

      integrator = new SimTK::RungeKuttaMersonIntegrator(osimModel->getMultibodySystem());
      integrator->setAccuracy(1.0e-6);

      osimManager = new OpenSim::Manager(*osimModel); 
	  
      //////////////
      const OpenSim::JointSet& joints = osimModel->getJointSet();
      int numJoints = osimModel->getNumJoints();

      /* OpenSim::Array< std::string > jointNames;
      
      std::cout << "joint ranges: " << std::endl;
      for (int u = 0; u < numJoints; u++)
	{
	  OpenSim::Joint *joint = &(joints[u]);	  
	  jointNames.append(joint->getName());
	    
	  OpenSim::CoordinateSet jointCoordinates = joint->getCoordinateSet();
	  OpenSim::Coordinate coordinate = jointCoordinates[0];
	  std::cout << coordinate.getRangeMin() << ", " << coordinate.getRangeMax()  << " --- " ;
	}
      std::cout << std::endl; */

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
	  
	  if (logger)
	  {
		///////////////////////
		// Pos/Vel/Aux logs
		
		// pos/vel
		unsigned int posVelSize = isi.getQ().size();
		std::string logWarning = "";
		if (isi.getQ().size() != isi.getU().size())
		{
			logWarning = "WARNING, size of Q (position) and U (velocities) vectors are not the same, which should not happen. Mistakes in logging could have happened.";
			std::cout << logWarning << std::endl;
			posVelSize = std::min( isi.getQ().size(), isi.getU().size() );
		}
		for (unsigned int u=0; u < posVelSize; u++)
		{
			std::string posvel_id;
			{
				std::stringstream tmpStr;
				tmpStr << "PosVel_" << u;
				posvel_id = tmpStr.str();
				
				tmpStr.str("");
				tmpStr << u;
		
				logger->newLogger(posvel_id, "position_and_velocity_" + tmpStr.str(),
					"# OpenSim/Simbody dynamics data log file\n# logged data from the Q (position) and U (velocities) vectors, entries number "+tmpStr.str()+" "+logWarning+"\n# data columns in this file:\n# time - position - velocity \n");
				
			}
		}
		
		// aux
		if (isi.getZ().size() > 0) // there are not always auxiliary variables in a system
		{
			std::stringstream tmpStr;
			for (unsigned int u=0; u < isi.getZ().size(); u++)
			{
				  tmpStr << "- auxiliary " << u;
			}
		
			logger->newLogger("Aux", "auxiliaries_",
				"# OpenSim/Simbody dynamics data log file\n# logged data from the Z (auxiliary) vector\n# data columns in this file:\n# time "+tmpStr.str()+" \n");
		}
		///////////////////////
		// Mobod log
		
		//osimModel->getMultibodySystem().realize(isi);
		int numMobilizedBodies = osimModel->getMatterSubsystem().getNumMobilities();

		for (int k = 0; k < numMobilizedBodies; ++k)
		{
			try
			{
				SimTK::MobodIndex mbi(k);
				if (mbi.isValid())
				{
					std::string mobod_id;
					{
					  std::stringstream tmpStr;
					  tmpStr << "Mobod_" << k;
					  mobod_id = tmpStr.str();
					}
				
					logger->newLogger(mobod_id, "velocities_and_accelerations_" + mobod_id,
					  "# OpenSim/Simbody dynamics data log file\n# logged data from "+mobod_id+"\n# data columns in this file:\n# time - angular velocity - translational velocity - angular acceleration - translational acceleration \n");
				}
			}
			catch (SimTK::Exception::CacheEntryOutOfDate& ex)
			{
			  std::clog << "Fallthrough exception querying MobilizedBody properties: " << ex.what() << std::endl;
			}
			catch (SimTK::Exception::StageTooLow& ex)
			{
			  std::clog << "Fallthrough exception querying MobilizedBody properties: " << ex.what() << std::endl;
			}
		}
		
		///////////////////////
		// rigid forces log
		try
		{
			const SimTK::Vector_<SimTK::SpatialVec>& rbForces = osimModel->getMultibodySystem().getRigidBodyForces(isi, isi.getSystemStage());
			for (int k = 0; k < rbForces.size(); ++k)
			{
				std::string rigidForce_id;
				{
					std::stringstream tmpStr;
					tmpStr << "RigidForce_" << k;
					rigidForce_id = tmpStr.str();
					
					tmpStr.str("");
					tmpStr << k;
				
					logger->newLogger(rigidForce_id, "rigid_body_forces_" + tmpStr.str(),
					  "# OpenSim/Simbody dynamics data log file\n# logged data from rigid force vector "+tmpStr.str()+"\n# data columns in this file:\n# time - torque - force \n");
				}
				
				//std::cout << " - " << k << ": " << rbForces[k] << std::endl;
			}
		}
		catch (SimTK::Exception::StageOutOfRange& ex)
		{
			std::clog << "Fallthrough exception retrieving Simbody RigidBody forces: " << ex.what() << std::endl;
		}
		
		///////////////////////
		// force set log
		const OpenSim::ForceSet& forces = osimModel->getForceSet();
		for (int u = 0; u < forces.getSize(); u++)
		{
			OpenSim::Force *force = &(forces[u]);
			OpenSim::Array<std::string> fLabels = force->getRecordLabels();

			std::string forceName = force->getName(); // this is the 'name' attribute of the force element in the .osim file
			if (force->getName().empty())
			{
				forceName = "unnamed force (set the 'name' attribute of the force in the .osim file)";
			}
			
			std::stringstream tmpStr;
			tmpStr << "Force_" << u;
			std::string force_id = tmpStr.str();
			
			tmpStr.str("");
			
			for (int m = 0; m < fLabels.size(); ++m)
			{
			  tmpStr << " - " << fLabels[m];
			}
			
			logger->newLogger(force_id, "force_set_" + force_id,
			  "# OpenSim/Simbody dynamics data log file\n# logged data from force named "+forceName+"\n# data columns in this file:\n# time"+tmpStr.str()+"\n");
		}
		
		
	  
		  //////////////
		  
		  if (osimModel->getMultibodySystem().hasContactSubsystem())
		  {
			const SimTK::GeneralContactSubsystem &contactSubsystem = osimModel->getMultibodySystem().getContactSubsystem();
			std::cout << "-- Found GeneralContactSubsystem containing " << contactSubsystem.getNumContactSets() << " contact sets --" << std::endl;

			for (int k = 0; k < contactSubsystem.getNumContactSets(); ++k)
			{
				SimTK::ContactSetIndex contactSetInd(k);
				if (contactSetInd.isValid())
				{
					std::cout << "   Contact set "<< k << " contains " << contactSubsystem.getNumBodies(contactSetInd) << " bodies, with transforms:   " << std::endl;
					
					for (int m = 0; m < contactSubsystem.getNumBodies(contactSetInd); ++m)
					{
						SimTK::ContactSurfaceIndex contactSurfaceInd(m);
						if (contactSurfaceInd.isValid())
						{
							std::cout << m << ": "<< contactSubsystem.getBody(contactSetInd, contactSurfaceInd).getBodyTransform(isi) << std::endl;
							
							if (logger)
							{
								///////////////////////
								// contact set log
								std::stringstream tmpStr;
								tmpStr << "ContactSet_" << k << "_Surface_" << m;
								std::string contactSetSurface_id = tmpStr.str();
								
								tmpStr.str("");
								
								/*tmpStr << "# OpenSim/Simbody dynamics data log file\n# logged data from a contact set with ContactSetIndex " << k << ", corresponding to contact mesh with ContactSurfaceIndex " << m << "\n# (note that in OpenSim a mesh-contact is always applied between the middle of a face of the intersecting mesh and the nearest point of the intersected object, which does not have to be a mesh itself)\n# data columns in this file:\n# time - contact face index - contact face distance - contact force normal\n";*/
								tmpStr << "# OpenSim/Simbody dynamics data log file\n# logged data from a contact set with ContactSetIndex " << k << ", corresponding to contact mesh with ContactSurfaceIndex " << m << "\n# (note that in OpenSim a mesh-contact is always applied between the middle of a face of the intersecting mesh and the nearest point of the intersected object, which does not have to be a mesh itself)\n# data columns in this file:\n# time - contact face index - position of contact point on intersecting mesh - position of contact point on intersected object - contact face distance - contact force normal\n";
								
								logger->newLogger(contactSetSurface_id, "contact_" + contactSetSurface_id, tmpStr.str());
								  
								  
								// fill ElasticFoundationForce_Parameters set with data (for contact point info calculation)
								SimTK::ContactSurfaceIndex bodyIndex = contactSurfaceInd;
								SimTK::GeneralContactSubsystem subsystem = contactSubsystem;
								SimTK::ContactSetIndex set=contactSetInd;
								
								if (subsystem.getBodyGeometry(set, bodyIndex).getTypeId() == SimTK::ContactGeometry::TriangleMesh::classTypeId())
								{
									SimTK_APIARGCHECK1(bodyIndex >= 0 && bodyIndex < subsystem.getNumBodies(set), "\"not actually a Simbody API method, but OpenSimSimulation\"", "Init",
											"Illegal body index: %d", (int)bodyIndex);
									SimTK_APIARGCHECK1(subsystem.getBodyGeometry(set, bodyIndex).getTypeId() 
														== SimTK::ContactGeometry::TriangleMesh::classTypeId(), 
										"\"not actually a Simbody API method, but OpenSimSimulation\"", "Init",
										"Body %d is not a triangle mesh", (int)bodyIndex);
									
									parameters[bodyIndex] = ElasticFoundationForce_Parameters();
									const SimTK::ContactGeometry::TriangleMesh& mesh = 
										SimTK::ContactGeometry::TriangleMesh::getAs
												(subsystem.getBodyGeometry(set, bodyIndex));
									ElasticFoundationForce_Parameters& param = parameters[bodyIndex];
									param.springPosition.resize(mesh.getNumFaces());
									param.springNormal.resize(mesh.getNumFaces());
									param.springArea.resize(mesh.getNumFaces());
									SimTK::Vec2 uv(SimTK::Real(1./3.), SimTK::Real(1./3.));
									for (int i = 0; i < (int) param.springPosition.size(); i++) 
									{
										param.springPosition[i] = 
										   (mesh.getVertexPosition(mesh.getFaceVertex(i, 0))
											+mesh.getVertexPosition(mesh.getFaceVertex(i, 1))
											+mesh.getVertexPosition(mesh.getFaceVertex(i, 2)))/3;
										param.springNormal[i] = -mesh.findNormalAtPoint(i, uv);
										param.springArea[i] = mesh.getFaceArea(i);
									}
								}
							}
						}
					}
				}
			}
		  }
		  else
		  {
			  std::cout << "-- MultibodySystem contains no GeneralContactSubsystem --" << std::endl;
		  }

	  } // if(logger)
	  
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
	
	// current positions are currently not logged, right now
    /* std::cout << "current Q (pos): " << ws.getQ() << std::endl;
    std::cout << "current U (vel): " << ws.getU() << std::endl;
    std::cout << "current Z (aux): " << ws.getZ() << std::endl; */
    

    bool status = osimManager->doIntegration(ws, 1, timeStep);
    if (status)
    {
      const SimTK::State& istate = osimManager->getIntegrator().getState();

      std::cout << "Stage after integration: " << istate.getSystemStage().getName() << std::endl;

      std::cout << "new Q (pos): " << istate.getQ() << std::endl;
      std::cout << "new U (vel): " << istate.getU() << std::endl;
      std::cout << "new Z (aux): " << istate.getZ() << std::endl;
	  unsigned int posVelSize = std::min( istate.getQ().size(), istate.getU().size() ); // just in case, those should always be the same, unless there is an object that has a position but no velocity or vice versa
	  for (unsigned int u=0; u < posVelSize; u++)
	  {
		  std::string posvel_id;
		  {
			  std::stringstream tmpStr;
			  tmpStr << "PosVel_" << u;
			  posvel_id = tmpStr.str();
			  
			  tmpStr.str("");
			  
			  tmpStr  << currentTime + timeStep
					<< " " << istate.getQ()[u] << " " << istate.getU()[u]
					<< "\n";
			  
			  logger->logData(posvel_id,tmpStr.str());
		  }
	  }
	  
	  {
			std::stringstream tmpStr;
			tmpStr  << currentTime + timeStep;
			for (unsigned int u=0; u < istate.getZ().size(); u++)
			{
				tmpStr  << " " << istate.getZ()[u];
			}
			tmpStr << "\n";
			logger->logData("Aux",tmpStr.str());
	  }

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

          std::cout << "   --> locked    : " << lci << ", isValid = " << lci.isValid() << std::endl;
          std::cout << "   --> clamped   : " << cci << ", isValid = " << cci.isValid() << std::endl;
          std::cout << "   --> prescribed: " << pci << ", isValid = " << pci.isValid() << std::endl;

          try
          {
            SimTK::Vector_<SimTK::SpatialVec> bfg;
            SimTK::Vector mf;
            if (lci.isValid())
            {
              const SimTK::Constraint& lc = osimModel->getMatterSubsystem().getConstraint(lci);
              lc.getConstraintForcesAsVectors(istate, bfg, mf);
              std::cout << "    --> locked: bfg = " << bfg << "; mf = " << mf << std::endl;
            }

            if (cci.isValid())
            {
              const SimTK::Constraint& cc = osimModel->getMatterSubsystem().getConstraint(cci);
              cc.getConstraintForcesAsVectors(istate, bfg, mf);
              std::cout << "    --> clamped: bfg = " << bfg << "; mf = " << mf << std::endl;
            }

            if (pci.isValid())
            {
              const SimTK::Constraint& pc = osimModel->getMatterSubsystem().getConstraint(pci);
              pc.getConstraintForcesAsVectors(istate, bfg, mf);
              std::cout << "    --> prescribed: bfg = " << bfg << "; mf = " << mf << std::endl;
            }
          }
          catch (SimTK::Exception::ErrorCheck& ex_err)
          {
            std::clog << "Fall-through exception for querying disabled Simbody Constraint instances: " << ex_err.what() << std::endl;
          }
        }
      }
      //jointReactionAnalysis->step(istate,1);

      /* std::cout << "new Q (pos): " << istate.getQ() << std::endl;
      std::cout << "new U (vel): " << istate.getU() << std::endl;
      std::cout << "new Z (aux): " << istate.getZ() << std::endl; */

      osimModel->getMultibodySystem().realize(ws);
      //int numMobilizedBodies = osimModel->getMatterSubsystem().getNumMobilities();
      int numMobilizedBodies = osimModel->getMatterSubsystem().getNumBodies();

      std::cout << "=== MobilizedBodies: " << numMobilizedBodies << " ===" << std::endl;
      for (int k = 0; k < numMobilizedBodies; ++k)
      {
        try
        {
          //const SimTK::State& dfs = osimManager->getIntegrator().getAdvancedState();
          SimTK::MobodIndex mbi(k);
		  
          if (mbi.isValid())
          {

            const SimTK::MobilizedBody& mb = osimModel->getMatterSubsystem().getMobilizedBody(mbi);
            const SimTK::SpatialVec& bta = mb.getBodyAcceleration(ws);
            const SimTK::SpatialVec& btv = mb.getBodyVelocity(ws);
            // const SimTK::Vec3& baa = mb.getBodyAngularAcceleration(ws);
			// const SimTK::Vec3& bav = mb.getBodyAngularVelocity(ws);
			
			/* std::cout << " - " << k << ": " << std::endl;
            std::cout << "  --> : translational velocity = " << btv[1] << "; translational acceleration = " << bta[1] << std::endl;
            //std::cout << "  --> : angular velocity: " << bav << "; angular acceleration: " << baa << std::endl;			
            std::cout << "  --> : angular velocity: " << btv[0] << "; angular acceleration: " << btv[1] << std::endl; */
			
			if (logger)
			{
				std::string mobod_id;
				{
				  std::stringstream tmpStr;
				  tmpStr << "Mobod_" << k;
				  mobod_id = tmpStr.str();
				  
				  tmpStr.str("");
				  
				  tmpStr  << currentTime + timeStep
						<< " " << btv[0][0] << " " << btv[0][1] << " " << btv[0][2]
						<< " " << btv[1][0] << " " << btv[1][1] << " " << btv[1][2]
						<< " " << bta[0][0] << " " << bta[0][1] << " " << bta[0][2]
						<< " " << bta[1][0] << " " << bta[1][1] << " " << bta[1][2]
						<< "\n";
				  
				  logger->logData(mobod_id,tmpStr.str());
				}
			}
			
          }
        }
        catch (SimTK::Exception::CacheEntryOutOfDate& ex)
        {
          std::clog << "Fallthrough exception querying MobilizedBody properties: " << ex.what() << std::endl;
        }
        catch (SimTK::Exception::StageTooLow& ex)
        {
          std::clog << "Fallthrough exception querying MobilizedBody properties: " << ex.what() << std::endl;
        }
      }

      try
      {
        const SimTK::Vector_<SimTK::SpatialVec>& rbForces = osimModel->getMultibodySystem().getRigidBodyForces(istate, istate.getSystemStage());
        std::cout << "=== Rigid body forces: " << rbForces.size() << " ===" << std::endl;
        for (int k = 0; k < rbForces.size(); ++k)
        {
          //std::cout << " - " << k << ": " << rbForces[k] << std::endl;
		  std::string rigidForce_id;
		  {
			  std::stringstream tmpStr;
			  tmpStr << "RigidForce_" << k;
			  rigidForce_id = tmpStr.str();
			  
			  tmpStr.str("");
			  
			  tmpStr  << currentTime + timeStep
					<< " " << rbForces[k][0][0] << " " << rbForces[k][0][1] << " " << rbForces[k][0][2]
					<< " " << rbForces[k][1][0] << " " << rbForces[k][1][1] << " " << rbForces[k][1][2]
					<< "\n";
			  
			  logger->logData(rigidForce_id,tmpStr.str());
		  }
        }
      }
      catch (SimTK::Exception::StageOutOfRange& ex)
      {
        std::clog << "Fallthrough exception retrieving Simbody RigidBody forces: " << ex.what() << std::endl;
      }

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
          std::cout << "     - condition: " << SimTK::Contact::nameOfCondition (contact.getCondition()) << std::endl;

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
          else if (SimTK::TriangleMeshContact::isInstance(contact))
          {
            std::cout << "     -> TriangleMeshContact." << std::endl;
            /* const SimTK::TriangleMeshContact& c = static_cast<const SimTK::TriangleMeshContact&>(contact); */
            const SimTK::TriangleMeshContact& c = SimTK::TriangleMeshContact::getAs(contact);
            std::cout << "        " << c << std::endl;
			
			//
            std::cout << "--- condition: " << SimTK::Contact::nameOfCondition (c.getCondition()) << std::endl;
            std::cout << "--- transform: " << c.getTransform() << std::endl;
			const std::set< int > &faces1 = c.getSurface1Faces();
			const std::set< int > &faces2 = c.getSurface2Faces();
			
			std::cout << "---   faces 1: ";
			for (std::set<int>::iterator it=faces1.begin(); it!=faces1.end(); ++it)
			{
				std::cout << ' ' << *it;
			}
			std::cout << std::endl;
			
			std::cout << "---   faces 2: ";
			for (std::set<int>::iterator it=faces2.begin(); it!=faces2.end(); ++it)
			{
				std::cout << ' ' << *it;
			}
			std::cout << std::endl;
			
			// tst Anfang
			if(logger)
			{
			// Aufruf von processContact in Simbody\src\ElasticFoundationForce.cpp
			/* processContact(state, contact.getSurface1(), 
                contact.getSurface2(), iter1->second, 
                contact.getSurface1Faces(), areaScale, bodyForces, pe); */
			
				
			if (logger)
			{
				std::cout << "--- contact info 1: " << std::endl;
				
				std::vector<std::string> logStringVec =
				calcContactInfo
			   (/* const SimTK::State& state */ istate, 
				/* SimTK::ContactSurfaceIndex meshIndex */ c.getSurface1(),
				/* SimTK::ContactSurfaceIndex otherBodyIndex */ c.getSurface2(), 
				parameters[c.getSurface1()],
				/* const std::set<int>& insideFaces */ c.getSurface1Faces(),
				// /*Real areaScale, Vector_<SpatialVec>& bodyForces, Real& pe,*/
				/* SimTK::GeneralContactSubsystem subsystem */ contactSystem,
				/* SimTK::ContactSetIndex set */ csi
				);
				
				std::stringstream tmpStr;
				tmpStr << "ContactSet_" << k << "_Surface_" << c.getSurface1();
				std::string contactSetSurface_id = tmpStr.str();
				
				tmpStr.str("");

				for (unsigned int a = 0; a < logStringVec.size(); a++)
				{
					tmpStr  << currentTime << " " << logStringVec.at(a) << "\n";
				}
				
				logger->logData(contactSetSurface_id,tmpStr.str());
			}
				
			if (logger)
			{
				
				std::cout << "--- contact info 2: " << std::endl;  // needs to switch surface 1 and 2
				std::vector<std::string> logStringVec = calcContactInfo
				   (/* const SimTK::State& state */ istate, 
					/* SimTK::ContactSurfaceIndex meshIndex */ c.getSurface2(),
					/* SimTK::ContactSurfaceIndex otherBodyIndex */ c.getSurface1(), 
					parameters[c.getSurface2()],
					/* const std::set<int>& insideFaces */ c.getSurface2Faces(),
					// /*Real areaScale, Vector_<SpatialVec>& bodyForces, Real& pe,*/
					/* SimTK::GeneralContactSubsystem subsystem */ contactSystem,
					/* SimTK::ContactSetIndex set */ csi
					);
					
				std::stringstream tmpStr;
				tmpStr << "ContactSet_" << k << "_Surface_" << c.getSurface2();
				std::string contactSetSurface_id = tmpStr.str();
				
				tmpStr.str("");

				for (unsigned int a = 0; a < logStringVec.size(); a++)
				{
					tmpStr  << currentTime << " " << logStringVec.at(a) << "\n";
				}
				
				logger->logData(contactSetSurface_id,tmpStr.str());
			}
				
			}
			// tst Ende
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

        /* for (int m = 0; m < fLabels.size(); ++m)
		{
          //std::cout << fLabels[m] << " = " << fValues[m] << ";";
          std::cout << fLabels[m] << " = " << fValues[m] << "\n";
		} */
		
		std::string force_id;
		{
			std::stringstream tmpStr;
			tmpStr << "Force_" << u;
			force_id = tmpStr.str();
			
			tmpStr.str("");
			
			tmpStr  << currentTime;
			for (int m = 0; m < fValues.size(); ++m)
			{
				tmpStr << " " << fValues[m];
			}
			tmpStr << "\n";
			
			logger->logData(force_id,tmpStr.str());
		}

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
	  
	  
	  if (osimModel->getMultibodySystem().hasContactSubsystem())
	  {
		const SimTK::GeneralContactSubsystem &contactSubsystem = osimModel->getMultibodySystem().getContactSubsystem();
		/* std::cout << "-- GeneralContactSubsystem currently contains the following contacts " << contactSubsystem.getContacts() << " --" << std::endl; */
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
        //std::cout << "=== Contact count in contactTracker: " << numc << " ===" << std::endl;
        std::cout << "=== Contact count in contactTracker: " << numc << " ===" << std::endl;
		std::cout << "-- contact tracker subsystem manages " << contactTracker->getNumSurfaces() << " surfaces --" << std::endl;
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

//void OpenSimSimulation::calcContactInfo
std::vector<std::string> OpenSimSimulation::calcContactInfo
   (const SimTK::State& state, 
    SimTK::ContactSurfaceIndex meshIndex, SimTK::ContactSurfaceIndex otherBodyIndex, 
    const ElasticFoundationForce_Parameters& param, const std::set<int>& insideFaces,
    /*Real areaScale, Vector_<SpatialVec>& bodyForces, Real& pe,*/
	SimTK::GeneralContactSubsystem subsystem,
	SimTK::ContactSetIndex set) const 
{
    const SimTK::ContactGeometry& otherObject = subsystem.getBodyGeometry(set, otherBodyIndex);
    const SimTK::MobilizedBody& body1 = subsystem.getBody(set, meshIndex);
    const SimTK::MobilizedBody& body2 = subsystem.getBody(set, otherBodyIndex);
    const SimTK::Transform t1g = body1.getBodyTransform(state)*subsystem.getBodyTransform(set, meshIndex); // mesh to ground
    const SimTK::Transform t2g = body2.getBodyTransform(state)*subsystem.getBodyTransform(set, otherBodyIndex); // other object to ground
    const SimTK::Transform t12 = ~t2g*t1g; // mesh to other object

    // Loop over all the springs, and evaluate the force from each one.

	std::vector<std::string> logVector;
	
    for (std::set<int>::const_iterator iter = insideFaces.begin(); 
                                       iter != insideFaces.end(); ++iter) {
										   
		std::stringstream logStringStream;
        int face = *iter;
        SimTK::UnitVec3 normal;
        bool inside;
        SimTK::Vec3 nearestPoint = otherObject.findNearestPoint(t12*param.springPosition[face], inside, normal);
        if (!inside)
            continue;
        
        // Find how much the spring is displaced.
        
        nearestPoint = t2g*nearestPoint;
        const SimTK::Vec3 springPosInGround = t1g*param.springPosition[face];
        const SimTK::Vec3 displacement = nearestPoint-springPosInGround;
        const SimTK::Real distance = displacement.norm();
        if (distance == 0.0)
            continue;
        const SimTK::Vec3 forceDir = displacement/distance;
		
		/* std::cout << "distance: " << distance << std::endl; 
		std::cout << "forceDir: " << forceDir << std::endl;  */
		std::cout << "face " << (*iter) << " springPosition: " << param.springPosition[face] << " nearestPoint: " << nearestPoint << " distance: " << distance << " forceDir: " << forceDir << std::endl;
		logStringStream << (*iter) 
						/* << " " << param.springPosition[face][0] << " " << param.springPosition[face][1] << " " << param.springPosition[face][2] */
						<< " " << springPosInGround[0] << " " << springPosInGround[1] << " " << springPosInGround[2]
						<< " " << nearestPoint[0] << " " << nearestPoint[1] << " " << nearestPoint[2]
						<< " " << distance
						<< " " << forceDir[0] << " " << forceDir[1] << " " << forceDir[2];
		logVector.push_back(logStringStream.str());
		
								
		/* std::cout << "normal?: " << normal << std::endl;  */ // Not entirely sure what this one is for
        
        // Calculate the relative velocity of the two bodies at the contact point.
        
        /* const SimTK::Vec3 station1 = body1.findStationAtGroundPoint(state, nearestPoint);
        const SimTK::Vec3 station2 = body2.findStationAtGroundPoint(state, nearestPoint);
        const SimTK::Vec3 v1 = body1.findStationVelocityInGround(state, station1);
        const SimTK::Vec3 v2 = body2.findStationVelocityInGround(state, station2);
        const SimTK::Vec3 v = v2-v1;
        const SimTK::Real vnormal = dot(v, forceDir);
        const SimTK::Vec3 vtangent = v-vnormal*forceDir; */
        
        // Calculate the damping force.
        
		// expand parameter class for this
		/* 
        const SimTK::Real area = areaScale * param.springArea[face];
        const SimTK::Real f = param.stiffness*area*distance*(1+param.dissipation*vnormal);
        SimTK::Vec3 force = (f > 0 ? f*forceDir : Vec3(0));
        
        // Calculate the friction force.
        
        const SimTK::Real vslip = vtangent.norm();
        if (f > 0 && vslip != 0) {
            const SimTK::Real vrel = vslip/transitionVelocity;
            const SimTK::Real ffriction = 
                f*(std::min(vrel, Real(1))
                 *(param.dynamicFriction+2*(param.staticFriction-param.dynamicFriction)
                 /(1+vrel*vrel))+param.viscousFriction*vslip);
            force += ffriction*vtangent/vslip;
        }

        body1.applyForceToBodyPoint(state, station1, force, bodyForces);
        body2.applyForceToBodyPoint(state, station2, -force, bodyForces);
        pe += param.stiffness*area*displacement.normSqr()/2;
		*/
    }
	
	return logVector;
}