#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

#include<cassert>
#include<memory>
#include<unordered_map>


class OSIMSIMULATION_API ExternalControlController : public Controller 
{
    OpenSim_DECLARE_CONCRETE_OBJECT(ExternalControlController, Controller);
  public:
        ExternalControlController();
        virtual ~ExternalControlController();

        /**
         * Compute the control values for all actuators under the control of this
         * Controller.
         *
         * @param s             system state 
         * @param controls      model controls  
         */
        void computeControls(const SimTK::State& s, 
                             SimTK::Vector& controls) const OVERRIDE_11;

        /**
         *      Assign a prescribe control value for the desired actuator identified
         *  by its name. Controller takes ownership of the function.
         *  @param actName                the actuator's name in the controller's set
         */
        void setControlValueForActuator(Actuator *actuator, double value);
        
protected:
        /** Model component interface */
        void connectToModel(Model& model) OVERRIDE_11;
        
        std::unordered_map<Actuator*, double> control_values;
};     


ExternalControlController::ExternalControlController() :
        Controller()
{
}

ExternalControlController::~ExternalControlController()
{
}


void ExternalControlController::connectToModel(Model& model)
{
        Super::connectToModel(model);
}


// compute the control value for an actuator
void ExternalControlController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
        SimTK::Vector actControls(1, 0.0);
        for(int i=0; i<getActuatorSet().getSize(); i++)
        {
          Actuator &act = getActuatorSet()[i];
          actControls[0] = const_cast<ExternalControlController*>(this)->control_values[&act];
          act.addInControls(actControls, controls);
        }  
}


void ExternalControlController::
        setControlValueForActuator(Actuator *actuator, double value)
{
  this->control_values[actuator] = value;
}



#define USE_MANAGER

int main()
{
  // Specify body mass of a 20 kg, 0.1m sides of cubed block body
  double blockMass = 20.0, blockSideLength = 0.1;
  // Constant distance of constraint to limit the block's motion
  double constantDistance = 0.2;
  // Contact parameters
  double stiffness = 1.0e7, dissipation = 0.1, friction = 0.2, viscosity=0.01;
  
  Object::clearArrayPropertiesOnUpdateFromXML = false;
  
  // Create an OpenSim model and set its name
  Model osimModel;
  osimModel.setName("tugOfWar");
  
#ifdef USE_MANAGER
  // Create the manager managing the forward integration and its outputs
  Manager manager(osimModel);
#endif
  
  // GROUND BODY
  // Get a reference to the model's ground body
  OpenSim::Body& ground = osimModel.getGroundBody();

  // Add display geometry to the ground to visualize in the Visualizer and GUI
  // add a checkered floor
  //ground.addDisplayGeometry("checkered_floor.vtp");
  // add anchors for the muscles to be fixed too
  ground.addDisplayGeometry("block.vtp");
  ground.addDisplayGeometry("block.vtp");

  {
  // block is 0.1 by 0.1 by 0.1m cube and centered at origin. 
  // transform anchors to be placed at the two extremes of the sliding block (to come)
  GeometrySet& geometry = ground.updDisplayer()->updGeometrySet();
  DisplayGeometry& anchor1 = geometry[1];
  //DisplayGeometry& anchor2 = geometry[2];
  // scale the anchors
  anchor1.setScaleFactors(Vec3(5, 1, 1));
  //anchor2.setScaleFactors(Vec3(5, 1, 1));
  // reposition the anchors
  anchor1.setTransform(Transform(Vec3(0, 0.05, 0.35)));
  //anchor2.setTransform(Transform(Vec3(0, 0.05, -0.35)));
  }
  // BLOCK BODY
  Vec3 blockMassCenter(0);
  Inertia blockInertia = blockMass*Inertia::brick(blockSideLength, blockSideLength, blockSideLength);

  // Create a new block body with the specified properties
  OpenSim::Body *block = new OpenSim::Body("block", blockMass, blockMassCenter, blockInertia);

  // Add display geometry to the block to visualize in the GUI
  block->addDisplayGeometry("block.vtp");

  // Create a new free joint with 6 degrees-of-freedom (coordinates) between the block and ground bodies
  Vec3 locationInParent(0, blockSideLength/2, 0), orientationInParent(0), locationInBody(0), orientationInBody(0);
  FreeJoint *blockToGround = new FreeJoint("blockToGround", ground, locationInParent, orientationInParent, *block, locationInBody, orientationInBody);
  
  // Get a reference to the coordinate set (6 degrees-of-freedom) between the block and ground bodies
  CoordinateSet& jointCoordinateSet = blockToGround->upd_CoordinateSet();

  // Set the angle and position ranges for the coordinate set
  double angleRange[2] = {-SimTK::Pi/2, SimTK::Pi/2};
  double positionRange[2] = {-1, 1};
  jointCoordinateSet[0].setRange(angleRange);
  jointCoordinateSet[1].setRange(angleRange);
  jointCoordinateSet[2].setRange(angleRange);
  jointCoordinateSet[3].setRange(positionRange);
  jointCoordinateSet[4].setRange(positionRange);
  jointCoordinateSet[5].setRange(positionRange);

  // Obtaine the default acceleration due to gravity
  Vec3 gravity = osimModel.getGravity();

  // Define non-zero default states for the free joint
  jointCoordinateSet[3].setDefaultValue(constantDistance); // set x-translation value
  double h_start = blockMass*gravity[1]/(stiffness*blockSideLength*blockSideLength);
  jointCoordinateSet[4].setDefaultValue(h_start); // set y-translation which is height

  // Add the block body to the model
  osimModel.addBody(block);

  // Define the initial and final simulation times
  double initialTime = 0.0;
  double finalTime = 3.00;

  // PRESCRIBED FORCE
  // Create a new prescribed force to be applied to the block
  PrescribedForce *prescribedForce = new PrescribedForce(block);
  prescribedForce->setName("prescribedForce");

  // Specify properties of the force function to be applied to the block
  double time[2] = {0, finalTime};                                      // time nodes for linear function
  double fXofT[2] = {0,  -blockMass*gravity[1]*3.0};    // force values at t1 and t2

  // Create linear function for the force components
  PiecewiseLinearFunction *forceX = new PiecewiseLinearFunction(2, time, fXofT);
  // Set the force and point functions for the new prescribed force
  prescribedForce->setForceFunctions(forceX, new Constant(0.0), new Constant(0.0));
  prescribedForce->setPointFunctions(new Constant(0.0), new Constant(0.0), new Constant(0.0));

  // Add the new prescribed force to the model
  osimModel.addForce(prescribedForce);
  
  /////////////////////////////////////////////
  // DEFINE CONSTRAINTS IMPOSED ON THE MODEL //
  /////////////////////////////////////////////
  Vec3 pointOnGround(0, blockSideLength/2 ,0);
  Vec3 pointOnBlock(0, 0, 0);

  // Create a new constant distance constraint
  ConstantDistanceConstraint *constDist = 
          new ConstantDistanceConstraint(ground, 
                  pointOnGround, *block, pointOnBlock, constantDistance);

  // Add the new point on a line constraint to the model
  osimModel.addConstraint(constDist);

  /////////////////////////////////////////////
  // CONTACT GEOMETRY AND FORCES              //
  /////////////////////////////////////////////
  // Define contact geometry
  // Create new floor contact halfspace
  ContactHalfSpace *floor = new ContactHalfSpace(SimTK::Vec3(0), SimTK::Vec3(0, 0, -0.5*SimTK_PI), ground, "floor");
  // Create new cube contact mesh
  OpenSim::ContactMesh *cube = new OpenSim::ContactMesh("blockMesh.obj", SimTK::Vec3(0), SimTK::Vec3(0), *block, "cube");

  // Add contact geometry to the model
  osimModel.addContactGeometry(floor);
  osimModel.addContactGeometry(cube);

  // Define contact parameters for elastic foundation force
  OpenSim::ElasticFoundationForce::ContactParameters *contactParams = 
          new OpenSim::ElasticFoundationForce::ContactParameters(stiffness, dissipation, friction, friction, viscosity);
  contactParams->addGeometry("cube");
  contactParams->addGeometry("floor");
  
  // Create a new elastic foundation (contact) force between the floor and cube.
  OpenSim::ElasticFoundationForce *contactForce = new OpenSim::ElasticFoundationForce(contactParams);
  contactForce->setName("contactForce");

  // Add the new elastic foundation force to the model
  osimModel.addForce(contactForce);
  
  ///////////////////////////////////////////
  // WANT TO LOAD MUSCLE SYSTEM FROM FILE! //
  ///////////////////////////////////////////
 
#if 0
  // MUSCLE FORCES
  // Create two new muscles with identical properties
  double maxIsometricForce = 1000.0, optimalFiberLength = 0.25, tendonSlackLength = 0.1, pennationAngle = 0.0; 
  Thelen2003Muscle *muscle1 = new Thelen2003Muscle("muscle1",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);
  Thelen2003Muscle *muscle2 = new Thelen2003Muscle("muscle2",maxIsometricForce,optimalFiberLength,tendonSlackLength,pennationAngle);

  // Specify the paths for the two muscles
  // Path for muscle 1
  muscle1->addNewPathPoint("muscle1-point1", ground, Vec3(0.0,0.05,-0.35));
  muscle1->addNewPathPoint("muscle1-point2", *block, Vec3(0.0,0.0,-0.05));
  // Path for muscle 2
  muscle2->addNewPathPoint("muscle2-point1", ground, Vec3(0.0,0.05,0.35));
  muscle2->addNewPathPoint("muscle2-point2", *block, Vec3(0.0,0.0,0.05));

  // Add the two muscles (as forces) to the model
  osimModel.addForce(muscle1);
  osimModel.addForce(muscle2);
#endif
  const char *files[2] = { "tug_of_war_muscle_definition1.osim", "tug_of_war_muscle_definition2.osim" };
  for (const char* fn : files)
  {
    std::cout << "Loading " << fn << std::endl;
    std::unique_ptr<XMLDocument> document(new XMLDocument(fn));
    SimTK::Xml::Element myNode =  document->getRootDataElement(); //either actual root or node after OpenSimDocument
    osimModel.updateFromXMLNode(myNode, document->getDocumentVersion());
  }

  ///////////////////////////////////
  // DEFINE CONTROLS FOR THE MODEL //
  ///////////////////////////////////
#if 0
  // Create a prescribed controller that simply applies controls as function of time
  // For muscles, controls are normalized motor-neuron excitations
  PrescribedController *muscleController = new PrescribedController();
  muscleController->setActuators(osimModel.updActuators());
  // Define linear functions for the control values for the two muscles
  Array<double> slopeAndIntercept1(0.0, 2);  // array of 2 doubles
  Array<double> slopeAndIntercept2(0.0, 2);
  // muscle1 control has slope of -1 starting 1 at t = 0
  slopeAndIntercept1[0] = -1.0/(finalTime-initialTime);  slopeAndIntercept1[1] = 1.0;
  // muscle2 control has slope of 0.95 starting 0.05 at t = 0
  slopeAndIntercept2[0] = 0.95/(finalTime-initialTime);  slopeAndIntercept2[1] = 0.05;
  
  // Set the indiviudal muscle control functions for the prescribed muscle controller
  muscleController->prescribeControlForActuator("fatigable", new LinearFunction(slopeAndIntercept1));
  muscleController->prescribeControlForActuator("original", new LinearFunction(slopeAndIntercept2));

  // Add the muscle controller to the model
  osimModel.addController(muscleController);
#endif

  ///////////////////////////////////
  // SPECIFY MODEL DEFAULT STATES  //
  ///////////////////////////////////
  
  auto* muscle1 = dynamic_cast<Millard2012EquilibriumMuscle*>(&osimModel.updForceSet().get("fatigable"));
  auto* muscle2 = dynamic_cast<Millard2012EquilibriumMuscle*>(&osimModel.updForceSet().get("original"));

  assert(muscle1 != nullptr);
  assert(muscle2 != nullptr);
  Actuator* muscles[2] = {
    muscle1, muscle2
  };
  
  
#if 1
  // Create a prescribed controller that simply applies controls as function of time
  // For muscles, controls are normalized motor-neuron excitations
  ExternalControlController *muscleController = new ExternalControlController();
  muscleController->setActuators(osimModel.updActuators());
  // Add the muscle controller to the model
  osimModel.addController(muscleController);
#endif
  
  //////////////////////////
  // PERFORM A SIMULATION //
  //////////////////////////

  // set use visualizer to true to visualize the simulation live
  osimModel.setUseVisualizer(true);

  // Initialize the system and get the default state
  SimTK::State& si = osimModel.initSystem();
  // Print out details of the model
  osimModel.printDetailedInfo(si, cout);
  
  // Enable constraint consistent with current configuration of the model
  constDist->setDisabled(si, false);

  cout << "Start height = "<< h_start << endl;
  osimModel.getMultibodySystem().realize(si, Stage::Velocity);

  // Compute initial conditions for muscles
  osimModel.equilibrateMuscles(si);

  double mfv1 = muscle1->getFiberVelocity(si);
  double mfv2 = muscle2->getFiberVelocity(si);

  // Create the force reporter for obtaining the forces applied to the model
  // during a forward simulation
  ForceReporter* reporter = new ForceReporter(&osimModel);
  osimModel.addAnalysis(reporter);

  SimTK::RungeKuttaMersonIntegrator* integrator = new SimTK::RungeKuttaMersonIntegrator(osimModel.getMultibodySystem());
  integrator->setAccuracy(1.0e-6);
  
#ifdef USE_MANAGER
  manager.setIntegrator(integrator);
#else
  // Create the integrator for integrating system dynamics
  SimTK::TimeStepper ts(osimModel.getMultibodySystem(), *integrator);
  ts.initialize(si);
  ts.setReportAllSignificantStates(true);
#endif
  
  double t = 0;
  const double dt = 0.01;
  for (int iter = 0; iter < 100; ++iter, t += dt)
  {
    osimModel.getMultibodySystem().realize(si, SimTK::Stage::Acceleration);
    const SimTK::Vector& controls = osimModel.getControls(si);
    std::cout << "t = " << iter << " ctrl = " << controls << std::endl;

#ifdef USE_MANAGER
    manager.setInitialTime(t);
    manager.setFinalTime(t + dt);
    manager.integrate(si);
    for (int i=0; i<2; ++i)
    {
      double a = std::sin(t * 3.14 + i * 1.57);
      muscleController->setControlValueForActuator(muscles[i], a*a);
    }
#else
    // NOTE: Not working ... :-(
//     SimTK::Vector& controls = osimModel.updControls(si);
//     
//     for (int i=0; i<controls.size(); ++i)
//     {
//       double a = std::sin(t * 3.14 + i * 1.57);
//       controls[i] = a*a;
//     }
    if(osimModel.isControlled())
      osimModel.updControllerSet().storeControls(si, iter);
    SimTK::Integrator::SuccessfulStepStatus status = ts.stepTo(t+dt);
#endif
  }

  return 0;
}
