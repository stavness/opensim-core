#ifndef _MultiController_h_
#define _MultiController_h_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  MultiController.h                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
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
#include "StretchController.h"

using namespace std;
namespace OpenSim {

static const double LENGTH_SET_POINT{ 0.33 };
static const double LENGTH_GAIN = 1.0;

//------------------------------------------------------------------------------
// MultiController
//------------------------------------------------------------------------------
class MultiController : public Controller {
	OpenSim_DECLARE_CONCRETE_OBJECT(MultiController, Controller);
public:
	// List of stretch controllers
	OpenSim_DECLARE_LIST_PROPERTY(controller_reps, StretchController, "list of stretch controllers");

	// Set point for all sub-controllers 
	OpenSim_DECLARE_INPUT(setpoint, double, SimTK::Stage::Model,
		"The set point");

	MultiController() {
		constructProperties();
	}

	void computeControls(const SimTK::State& s,
		SimTK::Vector& controls) const override
	{
		for (int i = 0; i < getProperty_controller_reps().size(); ++i) {
			get_controller_reps(i).computeControls(s, controls);
		}
	}

private:
	void constructProperties() {
		constructProperty_controller_reps();
	}

	void extendConnectToModel(OpenSim::Model& model) {
		Super::extendConnectToModel(model);

		const Input<double>& in = getInput<double>("setpoint");
		const Output<double>::Channel& out = in.getChannel();

		for (int i = 0; i < getProperty_controller_reps().size(); ++i) {
			auto con = get_controller_reps(i);
			auto pa = con.getConnectee<PathActuator>("actuator");
			con.updInput("fiberLength")
				.connect(pa.getOutput("length"));
			con.updInput("fiberLength_setpoint")
				.connect(out, "setpoint");
		}
	}

	// For sub-component
	//void extendConnectToModel(OpenSim::Model& model) {
	//	Super::extendConnectToModel(model);

	//	OPENSIM_THROW_IF_FRMOBJ(
	//		!dynamic_cast<const MultiController*>(&getParent()),
	//		Exception, "This compoment must be a subcomponent of "
	//		"MultiController.");
	//}

}; // end of MultiController

} // end of namespace OpenSim

#endif // _MultiController_h_
