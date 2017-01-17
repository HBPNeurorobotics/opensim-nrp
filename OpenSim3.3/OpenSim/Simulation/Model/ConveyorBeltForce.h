#ifndef OPENSIM_CONVEYOR_BELT_FORCE_H_
#define OPENSIM_CONVEYOR_BELT_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ConveyorBeltForce.h                          *
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
// INCLUDE
#include "OpenSim/Common/Property.h"
#include "OpenSim/Common/Set.h"

#include "Force.h"

namespace SimTK{
  // Implementation of a Simbody custom force Implementation class.
  class ConveyorBeltForceImpl: public SimTK::Force::Custom::Implementation {
  public:

    class Parameters {
    public:
    Parameters() : stiffness(1), dissipation(0), staticFriction(0), dynamicFriction(0), viscousFriction(0) {
      }
    Parameters(Real stiffness, Real dissipation, Real staticFriction, Real dynamicFriction, Real viscousFriction) :
      stiffness(stiffness), dissipation(dissipation), staticFriction(staticFriction), dynamicFriction(dynamicFriction), viscousFriction(viscousFriction) {
      }
      Real stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction;
      Array_<Vec3> springPosition;
      Array_<UnitVec3> springNormal;
      Array_<Real> springArea;
    };

  private:
    friend class ConveyorBeltForce;
    const GeneralContactSubsystem& subsystem;
    const ContactSetIndex set;
    std::map<ContactSurfaceIndex, Parameters> parameters;
    Real transitionVelocity;
	Vec3 conveyorForceDirection;
    mutable CacheEntryIndex energyCacheIndex;
	mutable bool conveyorForceNotAddedThisStep;
	mutable bool conveyorIntersectionHappened;

  public:
    
    ConveyorBeltForceImpl(SimTK::GeneralContactSubsystem& contacts, SimTK::ContactSetIndex set);

    void setBodyParameters
      (ContactSurfaceIndex bodyIndex, Real stiffness, Real dissipation, 
       Real staticFriction, Real dynamicFriction, Real viscousFriction);

    
    virtual void calcForce(const SimTK::State&         state, 
                           SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInG, 
                           SimTK::Vector_<SimTK::Vec3>&       particleForcesInG, 
                           SimTK::Vector&              mobilityForces) const ;
    
    void processContact(const State& state, ContactSurfaceIndex meshIndex, 
                        ContactSurfaceIndex otherBodyIndex, 
                        const Parameters& param, 
                        const std::set<int>& insideFaces,
                        Real areaScale,
                        Vector_<SpatialVec>& bodyForces, Real& pe) const;
    
    //virtual SimTK::Real calcPotentialEnergy(const SimTK::State& state) const ;
    Real calcPotentialEnergy(const State& state) const;
    void realizeTopology(State& state) const;
    
  };
 
  // Implementation of a Simbody handle class. Note: a handle class
  // may *not* have any data members or virtual methods.
  class ConveyorBeltForce : public SimTK::Force::Custom {
  public:
  ConveyorBeltForce(SimTK::GeneralForceSubsystem& forces, // note the "&"
		    SimTK::GeneralContactSubsystem& contacts,
		    SimTK::ContactSetIndex set) 
    :   SimTK::Force::Custom(forces, new SimTK::ConveyorBeltForceImpl(contacts, set)) {}

    void setBodyParameters
      (ContactSurfaceIndex bodyIndex, Real stiffness, Real dissipation, 
       Real staticFriction, Real dynamicFriction, Real viscousFriction) {
      //updImpl().setBodyParameters(bodyIndex, stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction);
      static_cast<ConveyorBeltForceImpl&>(updImplementation()).setBodyParameters(bodyIndex, stiffness, dissipation, staticFriction, dynamicFriction, viscousFriction);
    }

    Real getTransitionVelocity() /*const*/ {
      //return getImpl().transitionVelocity;
      return static_cast<ConveyorBeltForceImpl&>(updImplementation()).transitionVelocity;
    }

    void setTransitionVelocity(Real v) {
      //updImpl().transitionVelocity = v;
      static_cast<ConveyorBeltForceImpl&>(updImplementation()).transitionVelocity = v;
    }

    Vec3 getConveyorForceDirection() /*const*/ {
      return static_cast<ConveyorBeltForceImpl&>(updImplementation()).conveyorForceDirection;
    }

    void setConveyorForceDirection(Vec3 d) {
      static_cast<ConveyorBeltForceImpl&>(updImplementation()).conveyorForceDirection = d;
    }

    bool getConveyorIntersectionHappened() /*const*/ {
      return static_cast<ConveyorBeltForceImpl&>(updImplementation()).conveyorIntersectionHappened;
    }

    void setConveyorIntersectionHappened(bool h) {
      static_cast<ConveyorBeltForceImpl&>(updImplementation()).conveyorIntersectionHappened = h;
    }
  };  


} // namespace SimTK


namespace OpenSim {

  class Model;
  //==============================================================================
  //                       CONVEYOR BELT FORCE
  //==============================================================================
  /** This Force subclass implements an elastic foundation contact model. It 
      places a spring at the center of each face of each ContactMesh it acts on.  
      Those springs interact with all objects (both meshes and other objects) the 
      mesh comes in contact with.

      @author (ElasticFoundationForce: Peter Eastman) **/
  class OSIMSIMULATION_API ConveyorBeltForce : public Force {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConveyorBeltForce, Force);
  public:
    class ContactParameters;
    class ContactParametersSet;
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
	These are the serializable properties associated with this class. Others
	are inherited from the superclass. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(contact_parameters, 
			     ConveyorBeltForce::ContactParametersSet,
			     "Material properties.");
    OpenSim_DECLARE_PROPERTY(transition_velocity, double,
			     "Slip velocity (creep) at which peak static friction occurs.");
    OpenSim_DECLARE_PROPERTY(conveyor_force_direction, SimTK::Vec3,
			     "Direction of the force that is applied to colliding objects.");
    OpenSim_DECLARE_PROPERTY(conveyor_intersection_happened, bool,
			     "Don't touch this.");
    /**@}**/


    //==============================================================================
    // PUBLIC METHODS
    //==============================================================================
    ConveyorBeltForce();
    explicit ConveyorBeltForce(ContactParameters* params);

    /**
     * Create a SimTK::Force which implements this Force.
     */
    void addToSystem(SimTK::MultibodySystem& system) const;
    ContactParametersSet& updContactParametersSet();
    const ContactParametersSet& getContactParametersSet();

    /** Takes over ownership of the passed-in object. **/
    void addContactParameters(ContactParameters* params);
    /**
     * Get the transition velocity for switching between static and dynamic friction.
     */
    double getTransitionVelocity() const;
    /**
     * Set the transition velocity for switching between static and dynamic friction.
     */
    void setTransitionVelocity(double velocity);
    /**
     * 
     */
    SimTK::Vec3 getConveyorForceDirection() const;
    /**
     * 
     */
    void setConveyorForceDirection(SimTK::Vec3 direction);
    /**
     * 
     */
    bool getConveyorIntersectionHappened() const;
    /**
     * 
     */
    void setConveyorIntersectionHappened(bool happened);

    /**
     * Access to ContactParameters. Methods assume size 1 of ContactParametersSet and add one ContactParameter if needed
     */
    double getStiffness() ;
    void setStiffness(double stiffness) ;
    double getDissipation() ;
    void setDissipation(double dissipation);
    double getStaticFriction() ;
    void setStaticFriction(double friction);
    double getDynamicFriction() ;
    void setDynamicFriction(double friction);
    double getViscousFriction() ;
    void setViscousFriction(double friction);
    void addGeometry(const std::string& name);
    
    //-----------------------------------------------------------------------------
    // Reporting
    //-----------------------------------------------------------------------------
    /** 
     * Provide name(s) of the quantities (column labels) of the force value(s) to be reported
     */
    virtual OpenSim::Array<std::string> getRecordLabels() const ;
    /**
     *  Provide the value(s) to be reported that correspond to the labels
     */
    virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const ;
  private:
    // INITIALIZATION
    void constructProperties();

    //==============================================================================
  };	// END of class ConveyorBeltForce
  //==============================================================================
  //==============================================================================

#ifndef SWIG
  //==============================================================================
  //              CONVEYOR BELT FORCE :: CONTACT PARAMETERS
  //==============================================================================

  class OSIMSIMULATION_API ConveyorBeltForce::ContactParameters 
    :   public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConveyorBeltForce::ContactParameters, 
				    Object);
  public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
	These are the serializable properties associated with this class. Others
	are inherited from the superclass. **/
    /**@{**/
    OpenSim_DECLARE_LIST_PROPERTY(geometry, std::string,
				  "Names of geometry objects affected by these parameters.");

    OpenSim_DECLARE_PROPERTY(stiffness, double,
			     "");
    OpenSim_DECLARE_PROPERTY(dissipation, double,
			     "");
    OpenSim_DECLARE_PROPERTY(static_friction, double,
			     "");
    OpenSim_DECLARE_PROPERTY(dynamic_friction, double,
			     "");
    OpenSim_DECLARE_PROPERTY(viscous_friction, double,
			     "");
    /**@}**/

    //==============================================================================
    // PUBLIC METHODS
    //==============================================================================
    ContactParameters();
    ContactParameters(double stiffness, double dissipation, 
                      double staticFriction, double dynamicFriction, 
                      double viscousFriction);

    const Property<std::string>& getGeometry() const;
    Property<std::string>& updGeometry();
    void addGeometry(const std::string& name);
    double getStiffness() const;
    void setStiffness(double stiffness);
    double getDissipation() const;
    void setDissipation(double dissipation);
    double getStaticFriction() const;
    void setStaticFriction(double friction);
    double getDynamicFriction() const;
    void setDynamicFriction(double friction);
    double getViscousFriction() const;
    void setViscousFriction(double friction);

  private:
    void constructProperties();
  };


  //==============================================================================
  //             CONVEYOR BELT FORCE :: CONTACT PARAMETERS SET
  //==============================================================================

  class OSIMSIMULATION_API ConveyorBeltForce::ContactParametersSet 
    :   public Set<ConveyorBeltForce::ContactParameters> {
    OpenSim_DECLARE_CONCRETE_OBJECT(ConveyorBeltForce::ContactParametersSet, 
				    Set<ConveyorBeltForce::ContactParameters>);

  public:
    ContactParametersSet();

  private:
    void setNull();
  };

#endif

} // end of namespace OpenSim

#endif // OPENSIM_CONVEYOR_BELT_FORCE_H_
