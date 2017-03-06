#ifndef OPENSIM_MobilityDiscreteForce_H_
#define OPENSIM_MobilityDiscreteForce_H_

#include "OpenSim/Common/Property.h"
#include "OpenSim/Common/Set.h"
#include "OpenSim/Simulation/Model/Force.h"

namespace OpenSim {

class Coordinate;
class Model;

class OSIMSIMULATION_API MobilityDiscreteForce : public Force {
OpenSim_DECLARE_CONCRETE_OBJECT(MobilityDiscreteForce, Force);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. Others
    are inherited from the superclass. **/
    /**@{**/
        OpenSim_DECLARE_OPTIONAL_PROPERTY(coordinate, std::string,
                "Name of the coordinate to which this force is applied.");
        OpenSim_DECLARE_PROPERTY(force_value, double,
                "The force");
    /**@}**/

//==============================================================================
// PUBLIC METHODS
//==============================================================================
        MobilityDiscreteForce(const std::string& coordinateName="");

        void setForce(double force_value);

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

protected:
        /** Setup this CoordinateLimitForce as part of the model.
            This were the existence of the coordinate to limit is checked. */ 
        void connectToModel(Model& aModel) OVERRIDE_11;
        /** Create the underlying Force that is part of the multibodysystem. */
        void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;


private:
        // INITIALIZATION
        void constructProperties();
        
        // Set the Coordinate pointer, and set the corresponding name property
        // to match.
        void setCoordinate(Coordinate* coordinate);
        Coordinate* getCoordinate() const;
        SimTK::ReferencePtr<Coordinate> _coord;
        SimTK::ForceIndex _simtk_force_idx;
//==============================================================================
};	// END of class MobilityDiscreteForce
//==============================================================================

}
#endif // OPENSIM_MobilityDiscreteForce_H_


