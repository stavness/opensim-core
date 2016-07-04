/* -------------------------------------------------------------------------- *
 *                  OpenSim:  testbedStretchController.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ian Stavness, Mohammad Shabani, Chris Dembia,                   *
 *			  Shrinidhi K. Lakshmikanth, Ajay Seth, Thomas Uchida             *
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
#include "helperMethods.h"
#include "StretchController.h"

static const double LENGTH_SETPOINT{ 0.33 };
static const double REPORTING_INTERVAL{ 0.2 };
static const std::string testbedAttachment1{"ground"};
static const std::string testbedAttachment2{"load"};

using namespace std;

namespace OpenSim {

// Forward declarations for methods defined in HelperMethods.h
Model buildTestbed(); 
StretchController* buildStretchController(const Muscle&); 

//------------------------------------------------------------------------------
// Add a SignalGenerator to the StretchController length set point
//------------------------------------------------------------------------------
void addSignalGeneratorToController(Controller& controller, Function& func)
{
    auto lengthSignalGen = new SignalGenerator();
	lengthSignalGen->setName("lengthSetPointGen");

	lengthSignalGen->set_function(func);
	controller.addComponent(lengthSignalGen);
	
	// Connect the signal generator's output signal to the controller's setpoint 
	controller.updInput("length_setpoint")
        .connect(lengthSignalGen->getOutput("signal"));
} 


//------------------------------------------------------------------------------
// Add a ConsoleReporter to a model for displaying outputs from a device.
//------------------------------------------------------------------------------
void addDeviceConsoleReporterToModel(Model& model, Controller& controller,
    const std::vector<std::string>& deviceOutputs)
{
    // Create a new ConsoleReporter. Set its name and reporting interval.
    auto reporter = new ConsoleReporter();
    reporter->setName(model.getName() + "_" + controller.getName() + "_results");
    reporter->set_report_time_interval(REPORTING_INTERVAL);

    // Loop through the desired device outputs and add them to the reporter.
    for (auto thisOutputName : deviceOutputs)
        reporter->updInput("inputs").connect(controller.getOutput(thisOutputName));

    // Add the reporter to the model.
    model.addComponent(reporter);
}

inline Model buildTestbed()
{
	using SimTK::Vec3;
	using SimTK::Inertia;

	//Physical properties of the model
	double ballMass = 1000;
	double ballRadius = 0.05;
	double anchorWidth = 0.05;

	// Create a new OpenSim model.
	auto testbed = Model();
	testbed.setName("testbed");
	testbed.setUseVisualizer(true);
	testbed.setGravity(Vec3(0));

	// Get a reference to the model's ground body and attach geometry to it
	Ground& ground = testbed.updGround();
	auto brick = new Brick(Vec3(anchorWidth, anchorWidth, 2 * anchorWidth));
	brick->setColor(SimTK::Blue);
	ground.attachGeometry(brick);


	// create a second body and attach a sphere geometry to it
	OpenSim::Body * ball = new OpenSim::Body("load",
		ballMass,
		Vec3(0),
		ballMass*SimTK::Inertia::sphere(ballRadius));

	auto sphere = new Sphere(ballRadius);
	sphere->setName("sphere");
	sphere->setColor(SimTK::Green);
	ball->attachGeometry(sphere);
	testbed.addBody(ball);

	// ball connected  to ground via a slider along X
	double maxIsometricForce = 100.0, optimalFiberLength = 0.25, tendonSlackLength = 0.1, pennationAngle = SimTK::Pi / 4;
	double xSinG = optimalFiberLength*cos(pennationAngle) + tendonSlackLength;

	//SliderJoint s = SliderJoint()
	auto slider = new SliderJoint("gndToLoad",
		ground,
		Vec3(anchorWidth / 2 + xSinG, 0, 0),
		Vec3(0),
		*ball,
		Vec3(0),
		Vec3(0));

	testbed.addJoint(slider);

	CoordinateSet& jointCoordinateSet = slider->upd_CoordinateSet();
	jointCoordinateSet[0].setName("tx");
	jointCoordinateSet[0].setDefaultValue(0.5);
	jointCoordinateSet[0].setRangeMin(0.0);
	jointCoordinateSet[0].setRangeMax(2.0);

	StepFunction& motion = StepFunction(
		0.05, //t0
		0.5, //t1
		0,    //f(t0)
		0.1); //f(t1)

	jointCoordinateSet[0].setPrescribedFunction(motion);
	jointCoordinateSet[0].setDefaultIsPrescribed(true);

	// create a muscle and add it to model
	Millard2012EquilibriumMuscle *muscle = new Millard2012EquilibriumMuscle("muscle",
		maxIsometricForce,
		optimalFiberLength,
		tendonSlackLength,
		pennationAngle);


	muscle->addNewPathPoint("muscle-point1", ground, Vec3(0));
	muscle->addNewPathPoint("muscle-point2", *ball, Vec3(0));
	muscle->setName("muscle");
	testbed.addForce(muscle);


	testbed.setDebugLevel(0);
	return testbed;
}

} // namespace OpenSim



//------------------------------------------------------------------------------
// START HERE! Toggle "if (false)" to "if (true)" to enable/disable each step in
// the exercise. The project should execute without making any changes (you
// should see the unassisted hopper hop slightly).
//------------------------------------------------------------------------------
int main()
{
	try {
    
	using namespace OpenSim;
	using namespace std;

	// Build the testbed and controller.
	auto& testbed = buildTestbed();		
	auto& pathActuator = testbed.updComponent<PathActuator>("muscle");
	auto controller = buildStretchController(pathActuator);
	testbed.addController(controller);

	addSignalGeneratorToController(*controller, Constant(LENGTH_SETPOINT));

	// Show all Components in the testbed.
	showSubcomponentInfo(testbed);
	showSubcomponentInfo(*controller);

	// Add a ConsoleReporter 
	std::vector<std::string> controllerOutputs{ "stretch_control"};
	addDeviceConsoleReporterToModel(testbed, *controller, controllerOutputs);
	
	// Create the system, initialize the state, and simulate.
	SimTK::State& state = testbed.initSystem();
	simulate(testbed, state);

	}
	catch (const std::exception& e) {
		std::cout << e.what() << std::endl;
	}
	system("pause");
	return 0;

};
