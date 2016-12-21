/* -------------------------------------------------------------------------- *
 *                 OpenSim:  testSerializeOpenSimObjects.cpp                  *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testPropertiesDump(const OpenSim::Object& aObject);

static void indent(int nSpaces) {
    for (int i=0; i<nSpaces; ++i) cout << " ";
}

// Recursively dump out contents of an object and its properties.
static void dumpObj(const Object& obj, int nSpaces) {
    indent(nSpaces);
    cout << obj.getConcreteClassName() << " Object " 
         << (obj.getName().empty()?"NONAME":obj.getName())
         << endl;
    for (int p=0; p < obj.getNumProperties(); ++p) {
        const AbstractProperty& ap = obj.getPropertyByIndex(p); 
        indent(nSpaces+2);
        cout << ap.getName() << "=" << ap.toString() << endl;
        // Check return values from Property API for debugging purposes
        bool t1 = ap.isListProperty();
        bool t2 = ap.isObjectProperty();
        bool t3 = ap.isOneObjectProperty();
        bool t4 = ap.isOneValueProperty();
        string ts = ap.getTypeName();
        indent(nSpaces+2);
        cout << "isList, isObject, isOneObject, isOneValue, typeName = " <<
            t1 <<", "<< t2 <<", "<< t3 <<", "<< t4 <<", "<< ts << endl;
        if (ap.isObjectProperty()) {
            for (int i=0; i < ap.size(); ++i)
                dumpObj(ap.getValueAsObject(i), nSpaces+4);
        }
    }
}

static OpenSim::Object* randomize(OpenSim::Object* obj)
{
    if (obj==0) return 0; // maybe empty tag
    std::stringstream stream;
    stream << rand();
    obj->setName(obj->getConcreteClassName()+stream.str());
    // Cycle thru properties and based on type, populate with a random valid value
     for (int p=0; p < obj->getNumProperties(); ++p) {
        AbstractProperty& ap = obj->updPropertyByIndex(p); 
        //cout << ap.getName() << "=" << ap.toString() << endl;
        // Check return values from Property API for debugging purposes
        bool t1 = ap.isListProperty();
        bool t2 = ap.isObjectProperty();
        bool t3 = ap.isOneObjectProperty();
        bool t4 = ap.isOneValueProperty();
        string ts = ap.getTypeName();
        cout << ts << endl;
        if (ap.isOptionalProperty()) 
            continue;
        if (ts == "bool"&& !ap.isListProperty()) ap.updValue<bool>()= !ap.getValue<bool>();
        else if (ts == "integer"&& !ap.isListProperty()) ap.updValue<int>()=rand();
        else if (ts == "double" && !ap.isListProperty()) ap.updValue<double>()=(double) rand();
        else if (ts == "double" && ap.isListProperty() && ap.getMaxListSize() < 20){
            for (int i=0; i< ap.getMaxListSize(); ++i)
                ap.updValue<double>(i) = (double) rand();
        }
        else if (ap.isObjectProperty() && !ap.isListProperty())
            randomize(&ap.updValueAsObject(0));
        //else
        //    std::cout <<"Unrecognized Property" << std::endl;
       // std::cout << ap.toString() << std::endl;
     }
     return obj;
}
int main()
{
	try {
		Model testModel;
        srand((unsigned)time(0));
        // for Body, Joint, Constraint, Force, Marker, ContactGeometry, Controller and Probe
        ArrayPtrs<OpenSim::Body> availableBodyTypes;
        Object::getRegisteredObjectsOfGivenType<OpenSim::Body>(availableBodyTypes);
        for (int i=0; i< availableBodyTypes.getSize(); i++){
            Object* clone = availableBodyTypes[i]->clone();
            Object* randClone = randomize(clone);
            testModel.updBodySet().cloneAndAppend(*Body::safeDownCast(randClone));
        }
        ArrayPtrs<OpenSim::Force> availableForceTypes;
        Object::getRegisteredObjectsOfGivenType<OpenSim::Force>(availableForceTypes);
        for (int i=0; i< availableForceTypes.getSize(); i++){
            Object* clone = availableForceTypes[i]->clone();
            Object* randClone = randomize(clone);
            testModel.updForceSet().cloneAndAppend(*Force::safeDownCast(randClone));
        }
        ArrayPtrs<OpenSim::Constraint> availableConstraintTypes;
        Object::getRegisteredObjectsOfGivenType<OpenSim::Constraint>(availableConstraintTypes);
        for (int i=0; i< availableConstraintTypes.getSize(); i++){
            Object* clone = availableConstraintTypes[i]->clone();
            Object* randClone = randomize(clone);
            testModel.updConstraintSet().cloneAndAppend(*Constraint::safeDownCast(randClone));
        }
        /*
        ArrayPtrs<OpenSim::Controller> availableControllerTypes;
        Object::getRegisteredObjectsOfGivenType<OpenSim::Controller>(availableControllerTypes);
        for (int i=0; i< availableControllerTypes.getSize(); i++){
            Object* clone = availableControllerTypes[i]->clone();
            Object* randClone = randomize(clone);
            testModel.updControllerSet().cloneAndAppend(*Controller::safeDownCast(randClone));
        }*/
        ArrayPtrs<OpenSim::Probe> availableProbeTypes;
        Object::getRegisteredObjectsOfGivenType<OpenSim::Probe>(availableProbeTypes);
        for (int i=0; i< availableProbeTypes.getSize(); i++){
            Object* clone = availableProbeTypes[i]->clone();
            Object* randClone = randomize(clone);
            testModel.updProbeSet().cloneAndAppend(*Probe::safeDownCast(randClone));
        }
        
        testModel.print("allComponents.osim");
        Model deserializedModel("allComponents.osim", false);
        ASSERT(testModel==deserializedModel);
        
	}
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
	return 0;
}

void testPropertiesDump(const Object& aObject)
{
	dumpObj(aObject, 4);
}