/* -------------------------------------------------------------------------- *
 *                      OpenSim:  Mobility Discrete.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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

#include "MobilityDiscreteForce.h"

#include "OpenSim/Simulation/Model/Model.h"

namespace OpenSim {

//==============================================================================
//                          HUNT CROSSLEY FORCE
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, copy 
// assignment operator.

// Default constructor.
MobilityDiscreteForce::MobilityDiscreteForce(const std::string& coordinateName)
{
    constructProperties();
    if (!coordinateName.empty())
      set_coordinate(coordinateName);
}


void MobilityDiscreteForce::setForce(double force_value)
{
  set_force_value(force_value);
  if (this->_simtk_force_idx.isValid())
  {
    SimTK::Force& simtk_force_tmp = this->updModel().updForceSubsystem().updForce(_simtk_force_idx);
    SimTK::Force::MobilityDiscreteForce &simtk_force = static_cast<SimTK::Force::MobilityDiscreteForce&>(simtk_force_tmp);
    simtk_force.setMobilityForce(this->updModel().updWorkingState(), get_force_value());
  }
}

//_____________________________________________________________________________
/**
 * Sets the _model pointer to proper value
 * _coordinate is actually set inside _createSystem
 */
void MobilityDiscreteForce::connectToModel(Model& model)
{
    Super::connectToModel(model);
    _coord = &model.updCoordinateSet().get(get_coordinate());
}


void MobilityDiscreteForce::addToSystem(SimTK::MultibodySystem& system) const
{
    Super::addToSystem(system);
    
    SimTK::MobilizedBodyIndex idx(_coord->getBodyIndex());
    SimTK::Force::MobilityDiscreteForce simtk_force(_model->updForceSubsystem(), _model->getMatterSubsystem().getMobilizedBody(idx), get_force_value());

    // Beyond the const Component get the index so we can access the SimTK::Force later
    MobilityDiscreteForce* mutableThis = const_cast<MobilityDiscreteForce *>(this);
    mutableThis->_simtk_force_idx = simtk_force.getForceIndex();
}

void MobilityDiscreteForce::constructProperties()
{
  constructProperty_coordinate();
  constructProperty_force_value(0.0);
}


//=============================================================================
// Reporting
//=============================================================================
/** 
 * Provide names of the quantities (column labels) of the force value(s) reported
 * 
 */
OpenSim::Array<std::string> MobilityDiscreteForce::getRecordLabels() const 
{
	OpenSim::Array<std::string> labels("");
        labels.append(getName()+"_Force");
	return labels;
}
/**
 * Provide the value(s) to be reported that correspond to the labels
 */
OpenSim::Array<double> MobilityDiscreteForce::
getRecordValues(const SimTK::State& state) const 
{
	OpenSim::Array<double> values(1);
        // TODO: implement me
        values.append(0);
}

}// end of namespace OpenSim
