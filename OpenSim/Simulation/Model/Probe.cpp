// Probe.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHOR: Tim Dorn
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* Copyright (c)  2012 Stanford University
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "Probe.h"

using namespace std;
using namespace SimTK;


//This Measure returns a probe value value only at the Acceleration stage
template <class T>
class ProbeMeasure : public SimTK::Measure_<T> {
public:
    SimTK_MEASURE_HANDLE_PREAMBLE(ProbeMeasure, Measure_<T>);
 
    ProbeMeasure(Subsystem& sub, const OpenSim::Probe& probe)
    :   SimTK::Measure_<T>(sub, new Implementation(probe), AbstractMeasure::SetHandle()) {}
    SimTK_MEASURE_HANDLE_POSTSCRIPT(ProbeMeasure, Measure_<T>);
};
 
 
template <class T>
class ProbeMeasure<T>::Implementation : public SimTK::Measure_<T>::Implementation {
public:
    Implementation(const OpenSim::Probe& probe)
    :   SimTK::Measure_<T>::Implementation(1), m_probe(probe) {}
 
    // Default copy constructor, destructor, copy assignment are fine.
 
    // Implementations of virtual methods.
    Implementation* cloneVirtual() const {return new Implementation(*this);}
    int getNumTimeDerivativesVirtual() const {return 0;}
    Stage getDependsOnStageVirtual(int order) const
    {   return Stage::Acceleration; }
 
    void calcCachedValueVirtual(const State& s, int derivOrder, T& value) const
    {
        SimTK_ASSERT1_ALWAYS(derivOrder==0,
            "ProbeMeasure::Implementation::calcCachedValueVirtual():"
            " derivOrder %d seen but only 0 allowed.", derivOrder);
 
        Vector tmp = m_probe.computeProbeValue(s);
        value = tmp(0);

    }
private:
    const OpenSim::Probe& m_probe;
};




namespace OpenSim {

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
    Probe::Probe():ModelComponent() 
{
    setNull();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Probe::~Probe()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aProbe Probe to be copied.
 */
Probe::Probe(const Probe &aProbe) : ModelComponent(aProbe)
{
    setNull();
    copyData(aProbe);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Probe to another.
 *
 * @param aProbe Probe to be copied.
 */
void Probe::copyData(const Probe &aProbe)
{
    setPropertyValue("isDisabled", aProbe.getPropertyValue<bool>("isDisabled"));
    setPropertyValue("operation", aProbe.getPropertyValue<string>("operation"));
    setPropertyValue("operation_parameter", aProbe.getPropertyValue<double>("operation_parameter"));
    afterOperationValue = aProbe.afterOperationValue;
}


//_____________________________________________________________________________
/**
 * Set the data members of this Probe to their null values.
 */
void Probe::setNull(void)
{
    setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Probe::setupProperties(void)
{
    addProperty<bool>("isDisabled",
        "Flag indicating whether the Probe is disabled or not. Disabled"
        " means that the Probe will not be reported using the ProbeReporter.",
        false);

    addProperty<string>("operation",
        "The operation to perform on the probe value: "
        "''(no operation, just return the probe value), 'integrate', 'differentiate', 'scale'",
        "");

    addProperty<double>("operation_parameter",
        "For 'integrate', this represents the initial condition, for 'scale', this represents the scale factor.",
        0.0);
}

//_____________________________________________________________________________
/**
 * Create an underlying OpenSim::Probe
 */
void Probe::setup(Model& model)
{
    ModelComponent::setup(model);
}

//_____________________________________________________________________________
/**
 * Create the underlying system component(s).
 */
void Probe::createSystem(MultibodySystem& system) const
{
    ModelComponent::createSystem(system);

    Probe* mutableThis = const_cast<Probe*>(this);

    // Create a Measure of the value to be probed (operand).
    ProbeMeasure<SimTK::Real> beforeOperationValue(system, *this);  
    //Measure::Constant beforeOperationValue(system, 1);		// debug


    //cout << beforeOperationValue << endl;
    //std::system("pause");

    // Assign the correct (operation) Measure subclass to the operand
    // ----------------------------------------------------------------

    // Return the original probe value (no operation)
    if (getOperation() == "")
        mutableThis->afterOperationValue = beforeOperationValue;

    // Integrate the probe value
    // -----------------------------
    else if (getOperation() == "integrate") {
        Measure::Constant initCond(system, getOperationParameter());		// initial condition
        mutableThis->afterOperationValue = Measure::Integrate(system, beforeOperationValue, initCond);
    }

    // Differentiate the probe value
    // -----------------------------
    else if (getOperation() == "differentiate")
        mutableThis->afterOperationValue = Measure::Differentiate(system, beforeOperationValue);

    // Scale the probe value
    // -----------------------------
    else if (getOperation() == "scale")
        mutableThis->afterOperationValue = Measure::Scale(system, getOperationParameter(), beforeOperationValue);

    // Get the minimum of the probe value
    // ----------------------------------
    //else if (getOperation() == "minimum")
    //	mutableThis->afterOperationValue = Measure::Minimum(system, beforeOperationValue);

    // Get the maximum of the probe value
    // ----------------------------------
    //else if (getOperation() == "maximum")
    //	mutableThis->afterOperationValue = Measure::Maximum(system, beforeOperationValue);

    // Throw exception (invalid operation)
    // -------------------------------------
    else {
        string errorMessage = getConcreteClassName() + ": Invalid probe operation. Currently supports '', 'integrate', 'differentiate', and 'scale'.";
        throw (Exception(errorMessage.c_str()));
    }

}

//_____________________________________________________________________________
/**
 * Set defaults from state
 */
void Probe::setDefaultsFromState(const State& state)
{
    ModelComponent::setDefaultsFromState(state);

    setPropertyValue("isDisabled", isDisabled());
    setPropertyValue("operation", getOperation());
    setPropertyValue("operation_parameter", getOperation());
}


//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Probe& Probe::operator=(const Probe &aProbe)
{
    // BASE CLASS
    Object::operator=(aProbe);

    copyData(aProbe);

    return(*this);
}

//=============================================================================
// GET AND SET
//=============================================================================
//_____________________________________________________________________________
/**
 * Gets whether the Probe is disabled or not.
 *
 * @return true if the Probe is disabled, false if enabled.
 */
bool Probe::isDisabled() const
{
    return getPropertyValue<bool>("isDisabled");
}

//_____________________________________________________________________________
/**
 * Gets the operation being performed on the Probe value.
 *
 * @return operation string
 */
string Probe::getOperation() const
{
    return getPropertyValue<string>("operation");
}

//_____________________________________________________________________________
/**
 * Gets the operation parameter for the operation.
 *
 * @return operation_parameter double
 */
double Probe::getOperationParameter() const
{
    return getPropertyValue<double>("operation_parameter");
}

//_____________________________________________________________________________
/**
 * Sets whether the Probe is disabled or not.
 *
 */
void Probe::setDisabled(bool isDisabled) 
{
    setPropertyValue("isDisabled", isDisabled);
}

//_____________________________________________________________________________
/**
 * Sets the operation being performed on the Probe value.
 *
 */
void Probe::setOperation(string operation) 
{
    setPropertyValue("operation", operation);
}


//_____________________________________________________________________________
/**
 * Sets the operation_parameter for the operation.
 *
 */
void Probe::setOperationParameter(double operation_parameter) 
{
    setPropertyValue("operation_parameter", operation_parameter);
}


//=============================================================================
// REPORTING
//=============================================================================
//_____________________________________________________________________________
/** 
//_____________________________________________________________________________
/** 
 * Provide names of the probe value integrals (column labels) to be reported.
 */
Array<string> Probe::getRecordLabels() const 
{
    Array<string> labels;
    if (getOperation() == "")
        labels.append(getName());
    else if (getOperation() == "scale") {
        char n[10];
        sprintf(n, "%f", getOperationParameter());
        labels.append(getName()+"_"+getOperation()+"_"+n+"X");
    }
    else
        labels.append(getName()+"_"+getOperation());

    return labels;
}

//_____________________________________________________________________________
/**
 * Provide the probe values to be reported that correspond to the probe labels.
 */
Array<double> Probe::getRecordValues(const State& s) const 
{
    Array<double> values;
    values.append(afterOperationValue.getValue(s));

    return values;
}


} // end of namespace OpenSim