/** @file DigitalFilter.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/DigitalFilter.h>
#include <Golem/Demo/Common/Msg.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool DigitalFilter::create(const Desc& desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "DigitalFilter::create(): Invalid description");

	return true;
}

//------------------------------------------------------------------------------

bool FIIRFilter::create(const Desc& desc) {
	DigitalFilter::create(desc); // throws

	feedforwardCoeffs = desc.feedforwardCoeffs;
	feedbackCoeffs = desc.feedbackCoeffs;
	reset();
	return true;
}

void FIIRFilter::reset() {
	bInit = true;
}

void FIIRFilter::input(Real inputValue) {
	ASSERT(!Math::equals(feedbackCoeffs[0], REAL_ZERO, REAL_EPS))
	
	const U32 inputSize = U32(feedforwardCoeffs.size());
	const U32 outputSize = U32(feedbackCoeffs.size());
	
	if (bInit) {
		bInit = false;
		
		inputValues.resize(inputSize);
		outputValues.resize(outputSize);
		
		for (U32 i = 0; i < inputSize - 1; i++)
			inputValues[i] = inputValue;
		for (U32 i = 0; i < outputSize - 1; i++)
			outputValues[i] = inputValue;
	}

	Real outputValue = inputValue*feedforwardCoeffs[0];
	
	for (U32 i = inputSize - 1; i > 0; i--)
		outputValue += feedforwardCoeffs[i]*(inputValues[i] = inputValues[i - 1]);
	
	inputValues[0] = inputValue;

	for (U32 i = outputSize - 1; i > 0; i--)
		outputValue -= feedbackCoeffs[i]*(outputValues[i] = outputValues[i - 1]);

	outputValues[0] = outputValue/feedbackCoeffs[0];
}

Real FIIRFilter::output() const {
	return outputValues[0];
}

//------------------------------------------------------------------------------
