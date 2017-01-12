/* -------------------------------------------------------------------------- *
 *                    OpenSim:  ConveyorBeltForce.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): (ElasticFoundationForce: Peter Eastman)                         *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "ConveyorBeltForce.h"
#include "ContactGeometry.h"
#include "ContactGeometrySet.h"
#include "ContactMesh.h"
#include "Model.h"
#include <OpenSim/Simulation/Model/BodySet.h>

namespace SimTK{

  //SimTK_INSERT_DERIVED_HANDLE_DEFINITIONS(ConveyorBeltForce, ConveyorBeltForceImpl, Force);

  ConveyorBeltForceImpl::ConveyorBeltForceImpl
  (GeneralContactSubsystem& subsystem, ContactSetIndex set) : 
    subsystem(subsystem), set(set), transitionVelocity(Real(0.01)), conveyorForceDirection(Vec3(0,0,0)) {
    
  }

  void ConveyorBeltForceImpl::setBodyParameters
  (ContactSurfaceIndex bodyIndex, Real stiffness, Real dissipation, 
   Real staticFriction, Real dynamicFriction, Real viscousFriction) {
    SimTK_APIARGCHECK1(bodyIndex >= 0 && bodyIndex < subsystem.getNumBodies(set), "ConveyorBeltForceImpl", "setBodyParameters",
		       "Illegal body index: %d", (int)bodyIndex);
    SimTK_APIARGCHECK1(subsystem.getBodyGeometry(set, bodyIndex).getTypeId() 
		       == ContactGeometry::TriangleMesh::classTypeId(), 
		       "ConveyorBeltForceImpl", "setBodyParameters",
		       "Body %d is not a triangle mesh", (int)bodyIndex);
    parameters[bodyIndex] = 
      Parameters(stiffness, dissipation, staticFriction, dynamicFriction, 
		 viscousFriction);
    const ContactGeometry::TriangleMesh& mesh = 
      ContactGeometry::TriangleMesh::getAs
      (subsystem.getBodyGeometry(set, bodyIndex));
    Parameters& param = parameters[bodyIndex];
    param.springPosition.resize(mesh.getNumFaces());
    param.springNormal.resize(mesh.getNumFaces());
    param.springArea.resize(mesh.getNumFaces());
    Vec2 uv(Real(1./3.), Real(1./3.));
    for (int i = 0; i < (int) param.springPosition.size(); i++) {
      param.springPosition[i] = 
	(mesh.getVertexPosition(mesh.getFaceVertex(i, 0))
	 +mesh.getVertexPosition(mesh.getFaceVertex(i, 1))
	 +mesh.getVertexPosition(mesh.getFaceVertex(i, 2)))/3;
      param.springNormal[i] = -mesh.findNormalAtPoint(i, uv);
      param.springArea[i] = mesh.getFaceArea(i);
    }
    subsystem.invalidateSubsystemTopologyCache();
  }

  void ConveyorBeltForceImpl::calcForce
  (const State& state, Vector_<SpatialVec>& bodyForces, 
   Vector_<Vec3>& particleForces, Vector& mobilityForces) const 
  {
    conveyorForceNotAddedThisStep = true;
    
    const Array_<Contact>& contacts = subsystem.getContacts(state, set);
    Real& pe = Value<Real>::downcast
      (subsystem.updCacheEntry(state, energyCacheIndex));
    pe = 0.0;
    for (int i = 0; i < (int) contacts.size(); i++) {
      std::map<ContactSurfaceIndex, Parameters>::const_iterator iter1 = 
	parameters.find(contacts[i].getSurface1());
      std::map<ContactSurfaceIndex, Parameters>::const_iterator iter2 = 
	parameters.find(contacts[i].getSurface2());

      // If there are two meshes, scale each one's contributions by 50%.
      Real areaScale = (iter1==parameters.end() || iter2==parameters.end())
	? Real(1) : Real(0.5);

      if (iter1 != parameters.end()) {
	const TriangleMeshContact& contact = 
	  static_cast<const TriangleMeshContact&>(contacts[i]);
	processContact(state, contact.getSurface1(), 
		       contact.getSurface2(), iter1->second, 
		       contact.getSurface1Faces(), areaScale, bodyForces, pe);
      }

      if (iter2 != parameters.end()) {
	const TriangleMeshContact& contact = 
	  static_cast<const TriangleMeshContact&>(contacts[i]);
	processContact(state, contact.getSurface2(), 
		       contact.getSurface1(), iter2->second, 
		       contact.getSurface2Faces(), areaScale, bodyForces, pe);
      }
    }
  }

  void ConveyorBeltForceImpl::processContact
  (const State& state, 
   ContactSurfaceIndex meshIndex, ContactSurfaceIndex otherBodyIndex, 
   const Parameters& param, const std::set<int>& insideFaces,
   Real areaScale, Vector_<SpatialVec>& bodyForces, Real& pe) const 
  {
    const ContactGeometry& otherObject = subsystem.getBodyGeometry(set, otherBodyIndex);
    const MobilizedBody& body1 = subsystem.getBody(set, meshIndex);
    const MobilizedBody& body2 = subsystem.getBody(set, otherBodyIndex);
    const Transform t1g = body1.getBodyTransform(state)*subsystem.getBodyTransform(set, meshIndex); // mesh to ground
    const Transform t2g = body2.getBodyTransform(state)*subsystem.getBodyTransform(set, otherBodyIndex); // other object to ground
    const Transform t12 = ~t2g*t1g; // mesh to other object

    // Loop over all the springs, and evaluate the force from each one.

	bool oneInside = false;
    for (std::set<int>::const_iterator iter = insideFaces.begin(); 
	 iter != insideFaces.end(); ++iter) {
      int face = *iter;
      UnitVec3 normal;
      bool inside;
      Vec3 nearestPoint = otherObject.findNearestPoint(t12*param.springPosition[face], inside, normal);
      if (!inside)
	  {
		  continue;
	  }
        
      // Find how much the spring is displaced.
        
      nearestPoint = t2g*nearestPoint;
      const Vec3 springPosInGround = t1g*param.springPosition[face];
      const Vec3 displacement = nearestPoint-springPosInGround;
      const Real distance = displacement.norm();
      if (distance == 0.0)
	  {
		  continue;
	  }
      const Vec3 forceDir = displacement/distance;
        
	  oneInside = true;  // apply the conveyor force only if there is at least one point in contact
	  
      // Calculate the relative velocity of the two bodies at the contact point.
        
      const Vec3 station1 = body1.findStationAtGroundPoint(state, nearestPoint);
      const Vec3 station2 = body2.findStationAtGroundPoint(state, nearestPoint);
      const Vec3 v1 = body1.findStationVelocityInGround(state, station1);
      const Vec3 v2 = body2.findStationVelocityInGround(state, station2);
      const Vec3 v = v2-v1;
      const Real vnormal = dot(v, forceDir);
      const Vec3 vtangent = v-vnormal*forceDir;
        
      // Calculate the damping force.
        
      const Real area = areaScale * param.springArea[face];
      const Real f = param.stiffness*area*distance*(1+param.dissipation*vnormal);
      Vec3 force = (f > 0 ? f*forceDir : Vec3(0));
        
      // Calculate the friction force.
        
      const Real vslip = vtangent.norm();
      if (f > 0 && vslip != 0) {
	const Real vrel = vslip/transitionVelocity;
	const Real ffriction = 
	  f*(std::min(vrel, Real(1))
	     *(param.dynamicFriction+2*(param.staticFriction-param.dynamicFriction)
	       /(1+vrel*vrel))+param.viscousFriction*vslip);
	force += ffriction*vtangent/vslip;
      }

      //force +=  conveyorForceDirection;
	  //std::cout << "conveyorForceDirection: " << conveyorForceDirection << std::endl;

      body1.applyForceToBodyPoint(state, station1, force, bodyForces);
      body2.applyForceToBodyPoint(state, station2, -force, bodyForces);
      pe += param.stiffness*area*displacement.normSqr()/2;
    }
	
    if (conveyorForceNotAddedThisStep && oneInside)
    {
		std::cout << "Adding conveyor force." << std::endl;
      SpatialVec convForce;
      convForce[0] = Vec3(0,0,0);
      convForce[1] = conveyorForceDirection;
      body1.applyBodyForce(state , convForce , bodyForces);
      body2.applyBodyForce(state , convForce , bodyForces);
	  conveyorForceNotAddedThisStep = false;
    }
  }

  Real ConveyorBeltForceImpl::calcPotentialEnergy(const State& state) const {
    return Value<Real>::downcast
      (subsystem.getCacheEntry(state, energyCacheIndex));
  }

  void ConveyorBeltForceImpl::realizeTopology(State& state) const {
    energyCacheIndex = subsystem.allocateCacheEntry
      (state, Stage::Dynamics, new Value<Real>());
  }

} // namespace SimTK

namespace OpenSim {

  //==============================================================================
  //                         CONVEYOR BELT FORCE
  //==============================================================================

  // Default constructor.
  ConveyorBeltForce::ConveyorBeltForce()
  {
    constructProperties();
  }

  // Construct with supplied contact parameters.
  ConveyorBeltForce::ConveyorBeltForce(ContactParameters* params)
  {
    constructProperties();
    addContactParameters(params);
  }

  void ConveyorBeltForce::addToSystem(SimTK::MultibodySystem& system) const
  {
    Super::addToSystem(system);

    const ContactParametersSet& contactParametersSet = 
      get_contact_parameters();
    const double& transitionVelocity = get_transition_velocity();
    const SimTK::Vec3& conveyorForceDirection = get_conveyor_force_direction();

    SimTK::GeneralContactSubsystem& contacts = system.updContactSubsystem();
    SimTK::SimbodyMatterSubsystem& matter = system.updMatterSubsystem();
    SimTK::ContactSetIndex set = contacts.createContactSet();
    SimTK::ConveyorBeltForce force(_model->updForceSubsystem(), contacts, set);
    force.setTransitionVelocity(transitionVelocity);
    force.setConveyorForceDirection(conveyorForceDirection);
    for (int i = 0; i < contactParametersSet.getSize(); ++i)
      {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
	  {
	    if (!_model->updContactGeometrySet().contains(params.getGeometry()[j]))
	      {
                std::string errorMessage = "Invalid ContactGeometry (" + params.getGeometry()[j] + ") specified in ConveyorBeltForce" + getName();
		throw (Exception(errorMessage.c_str()));
	      }
	    ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
            contacts.addBody(set, matter.updMobilizedBody(SimTK::MobilizedBodyIndex(geom.getBody().getIndex())), geom.createSimTKContactGeometry(), geom.getTransform());
            if (dynamic_cast<ContactMesh*>(&geom) != NULL)
	      force.setBodyParameters(SimTK::ContactSurfaceIndex(contacts.getNumBodies(set)-1), 
				      params.getStiffness(), params.getDissipation(),
				      params.getStaticFriction(), params.getDynamicFriction(), params.getViscousFriction());
	  }
      }

    // Beyond the const Component get the index so we can access the SimTK::Force later
    ConveyorBeltForce* mutableThis = const_cast<ConveyorBeltForce *>(this);
    mutableThis->_index = force.getForceIndex();
  }

  void ConveyorBeltForce::constructProperties()
  {
    constructProperty_contact_parameters(ContactParametersSet());
    constructProperty_transition_velocity(0.01);
	constructProperty_conveyor_force_direction(SimTK::Vec3(0,0,0));
  }


  ConveyorBeltForce::ContactParametersSet& ConveyorBeltForce::updContactParametersSet()
  {
    return upd_contact_parameters();
  }

  const ConveyorBeltForce::ContactParametersSet& ConveyorBeltForce::getContactParametersSet()
  {
    return get_contact_parameters();
  }

  void ConveyorBeltForce::addContactParameters(ConveyorBeltForce::ContactParameters* params)
  {
    updContactParametersSet().adoptAndAppend(params);
  }

  double ConveyorBeltForce::getTransitionVelocity() const
  {
    return get_transition_velocity();
  }

  void ConveyorBeltForce::setTransitionVelocity(double velocity)
  {
    set_transition_velocity(velocity);
  }

  SimTK::Vec3 ConveyorBeltForce::getConveyorForceDirection() const
  {
    return get_conveyor_force_direction();
  }

  void ConveyorBeltForce::setConveyorForceDirection(SimTK::Vec3 direction)
  {
    set_conveyor_force_direction(direction);
  }
  
  /* The following set of functions are introduced for convenience to get/set values in ConveyorBeltForce::ContactParameters
   * and for access in Matlab without exposing ConveyorBeltForce::ContactParameters. pending refactoring contact forces
   */
  double ConveyorBeltForce::getStiffness()  { 
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    return get_contact_parameters().get(0).getStiffness(); 
  };
  void ConveyorBeltForce::setStiffness(double stiffness) {
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    upd_contact_parameters()[0].setStiffness(stiffness); 
  };
  double ConveyorBeltForce::getDissipation()   { 
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    return get_contact_parameters().get(0).getDissipation(); 
  };
  void ConveyorBeltForce::setDissipation(double dissipation) {
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    upd_contact_parameters()[0].setDissipation(dissipation); 
  };
  double ConveyorBeltForce::getStaticFriction()  { 
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    return get_contact_parameters().get(0).getStaticFriction(); 
  };
  void ConveyorBeltForce::setStaticFriction(double friction) {
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    upd_contact_parameters()[0].setStaticFriction(friction); 
  };
  double ConveyorBeltForce::getDynamicFriction()   { 
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    return get_contact_parameters().get(0).getDynamicFriction(); 
  };
  void ConveyorBeltForce::setDynamicFriction(double friction) {
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    upd_contact_parameters()[0].setDynamicFriction(friction); 
  };
  double ConveyorBeltForce::getViscousFriction()   { 
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    return get_contact_parameters().get(0).getViscousFriction(); 
  };
  void ConveyorBeltForce::setViscousFriction(double friction) {
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    upd_contact_parameters()[0].setViscousFriction(friction); 
  };
  
  void ConveyorBeltForce::addGeometry(const std::string& name)
  {
    if (get_contact_parameters().getSize()==0)
      updContactParametersSet().adoptAndAppend(new ConveyorBeltForce::ContactParameters());
    upd_contact_parameters()[0].addGeometry(name);
  }

  //==============================================================================
  //               CONVEYOR BELT FORCE :: CONTACT PARAMETERS
  //==============================================================================

  // Default constructor.
  ConveyorBeltForce::ContactParameters::ContactParameters()
  {
    constructProperties();
  }

  // Constructor specifying material properties.
  ConveyorBeltForce::ContactParameters::ContactParameters
  (double stiffness, double dissipation, double staticFriction, 
   double dynamicFriction, double viscousFriction)
  {
    constructProperties();
    set_stiffness(stiffness);
    set_dissipation(dissipation);
    set_static_friction(staticFriction);
    set_dynamic_friction(dynamicFriction);
    set_viscous_friction(viscousFriction);
  }


  void ConveyorBeltForce::ContactParameters::constructProperties()
  {
    constructProperty_geometry(); // a list of strings
    constructProperty_stiffness(0.0);
    constructProperty_dissipation(0.0);
    constructProperty_static_friction(0.0);
    constructProperty_dynamic_friction(0.0);
    constructProperty_viscous_friction(0.0);
  }

  const Property<std::string>& ConveyorBeltForce::ContactParameters::getGeometry() const
  {
    return getProperty_geometry();
  }

  Property<std::string>& ConveyorBeltForce::ContactParameters::updGeometry()
  {
    return updProperty_geometry();
  }

  void ConveyorBeltForce::ContactParameters::addGeometry(const std::string& name)
  {
    updGeometry().appendValue(name);
  }

  double ConveyorBeltForce::ContactParameters::getStiffness() const
  {
    return get_stiffness();
  }

  void ConveyorBeltForce::ContactParameters::setStiffness(double stiffness)
  {
    set_stiffness(stiffness);
  }

  double ConveyorBeltForce::ContactParameters::getDissipation() const
  {
    return get_dissipation();
  }

  void ConveyorBeltForce::ContactParameters::setDissipation(double dissipation)
  {
    set_dissipation(dissipation);
  }

  double ConveyorBeltForce::ContactParameters::getStaticFriction() const
  {
    return get_static_friction();
  }

  void ConveyorBeltForce::ContactParameters::setStaticFriction(double friction)
  {
    set_static_friction(friction);
  }

  double ConveyorBeltForce::ContactParameters::getDynamicFriction() const
  {
    return get_dynamic_friction();
  }

  void ConveyorBeltForce::ContactParameters::setDynamicFriction(double friction)
  {
    set_dynamic_friction(friction);
  }

  double ConveyorBeltForce::ContactParameters::getViscousFriction() const
  {
    return get_viscous_friction();
  }

  void ConveyorBeltForce::ContactParameters::setViscousFriction(double friction)
  {
    set_viscous_friction(friction);
  }

  void ConveyorBeltForce::ContactParametersSet::setNull()
  {
    setAuthors("(ElasticFoundationForce: Peter Eastman)");
  }

  ConveyorBeltForce::ContactParametersSet::ContactParametersSet()
  {
    setNull();
  }


  //=============================================================================
  // Reporting
  //=============================================================================
  /** 
   * Provide names of the quantities (column labels) of the force value(s) reported
   * 
   */
  OpenSim::Array<std::string> ConveyorBeltForce::getRecordLabels() const 
  {
    OpenSim::Array<std::string> labels("");

    const ContactParametersSet& contactParametersSet = 
      get_contact_parameters();

    for (int i = 0; i < contactParametersSet.getSize(); ++i)
      {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
	  {
	    ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
	    std::string bodyName = geom.getBodyName();
	    labels.append(getName()+"."+bodyName+".conveyor.force.X");
	    labels.append(getName()+"."+bodyName+".conveyor.force.Y");
	    labels.append(getName()+"."+bodyName+".conveyor.force.Z");
	    labels.append(getName()+"."+bodyName+".conveyor.torque.X");
	    labels.append(getName()+"."+bodyName+".conveyor.torque.Y");
	    labels.append(getName()+"."+bodyName+".conveyor.torque.Z");
	  }
      }

    return labels;
  }
  /**
   * Provide the value(s) to be reported that correspond to the labels
   */
  OpenSim::Array<double> ConveyorBeltForce::getRecordValues(const SimTK::State& state) const 
  {
    OpenSim::Array<double> values(1);

    const ContactParametersSet& contactParametersSet = 
      get_contact_parameters();

    const SimTK::ConveyorBeltForce& simtkForce = 
      (SimTK::ConveyorBeltForce &)(_model->getForceSubsystem().getForce(_index));

    SimTK::Vector_<SimTK::SpatialVec> bodyForces(0);
    SimTK::Vector_<SimTK::Vec3> particleForces(0);
    SimTK::Vector mobilityForces(0);

    //get the net force added to the system contributed by the Spring
    simtkForce.calcForceContribution(state, bodyForces, particleForces, mobilityForces);

    for (int i = 0; i < contactParametersSet.getSize(); ++i)
      {
        ContactParameters& params = contactParametersSet.get(i);
        for (int j = 0; j < params.getGeometry().size(); ++j)
	  {
	    ContactGeometry& geom = _model->updContactGeometrySet().get(params.getGeometry()[j]);
	    std::string bodyName = geom.getBodyName();
	
	    SimTK::Vec3 forces = bodyForces(_model->getBodySet().get(bodyName).getIndex())[1];
	    SimTK::Vec3 torques = bodyForces(_model->getBodySet().get(bodyName).getIndex())[0];

	    values.append(3, &forces[0]);
	    values.append(3, &torques[0]);
	  }
      }

    return values;
  }
  
} // end of namespace OpenSim
