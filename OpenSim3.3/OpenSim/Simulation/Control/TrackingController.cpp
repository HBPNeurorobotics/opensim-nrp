/* -------------------------------------------------------------------------- *
 *                      OpenSim:  TrackingController.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <cstdio>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "TrackingController.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>
#include <OpenSim/Simulation/Model/Actuator.h>
#include "SimTKsimbody.h"


//=============================================================================
// STATICS
//=============================================================================

using namespace OpenSim;
using namespace std;



//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
TrackingController::TrackingController() : Controller()
{
	setNull();
}


//_____________________________________________________________________________
/**
 * Copy constructor.
 */
TrackingController::TrackingController(const TrackingController &aTrackingController) :
	Controller(aTrackingController)
{
	setNull();
	copyData(aTrackingController);
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
TrackingController::~TrackingController()
{

}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set NULL values for all member variables.
 */
void TrackingController::setNull()
{
	setAuthors("Ajay Seth");
	setupProperties();
	_desiredStatesStorage = NULL;
	_trackingTasks = NULL;
}
//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void TrackingController::setupProperties()
{

}
//_____________________________________________________________________________
/**
 * Copy the member variables of the specified controller.
 */
void TrackingController::copyData(const TrackingController &aController)
{
	_desiredStatesStorage = aController._desiredStatesStorage;
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 */
TrackingController& TrackingController::operator=(const TrackingController &aController)
{
	// BASE CLASS
	Controller::operator=(aController);

	// DATA
	copyData(aController);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
void TrackingController::setDesiredStatesStorage(const Storage* aYDesStore)
{
	_desiredStatesStorage = aYDesStore;
}

const Storage& TrackingController:: getDesiredStatesStorage() const
{
	return *_desiredStatesStorage;
}

