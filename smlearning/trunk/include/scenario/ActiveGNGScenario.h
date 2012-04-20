/** @file PredictingScenario.h
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#ifndef SMLEARNING_ACTIVEGNGSCENARIO_H_
#define SMLEARNING_ACTIVEGNGSCENARIO_H_

#include <scenario/Scenario.h>
#include <metalearning/CrySSMEx.h>
#include <boost/function.hpp>

namespace smlearning {

//! \class ActiveGNGScenario
/*! \brief For active quantization by using Active GNG algorithm
 */
class ActiveGNGScenario : public Scenario {
public:
	/** Object description */
	class Desc : public Scenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(ActiveGNGScenario, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			Scenario::Desc::setToDefault();
		}
	};

	/** Objects can be constructed only in the Scene context. */
	ActiveGNGScenario(golem::Scene &scene);
	/** destructor */
	~ActiveGNGScenario ();
	/** get class name */
	static string getName () { return "ActiveGNGScenario"; }
	/** Run experiment */
	void run(int argc, char* argv[]);
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);
protected:
	// /** Renders the object. */
        // virtual void render();
	// /** (Post)processing function called AFTER every physics simulation step and before rendering. */
	// virtual void postprocess(golem::SecTmReal elapsedTime);
	/** Input Quantizer learning process for current sequence */
	void trainInputQuantizer (int iteration);
	/** Output Quantizer learning process for current sequence */
	void trainOutputQuantizer (int iteration);
	/** Wait for input quantizer to finish learning */
	void waitForInputQuantizer ();
	/** Wait for output quantizer to finish learning */
	void waitForOutputQuantizer ();
	/** save input quantizer */
	void saveInputQuantizer ();
	/** save output quantizer */
	void saveOutputQuantizer ();
	/** set data set if it was used for pretraining of quantizers */
	void setData (string seqFile);
	/** Encapsulation of CrySSMEx components */
	ActiveCrySSMEx cryssmex;
	/** method for feature selection */
	unsigned int featureSelectionMethod;
	/** current input data sequence (for quantization) */
	vector<FeatureVector> currentInputSeq;
	/** current output data sequence (for quantization) */
	vector<FeatureVector> currentOutputSeq;
	/** normalization function */
	boost::function<float (const float&, const float&, const float&)> normalization;
	/** denormalization function */
	boost::function<float (const float&, const float&, const float&)> denormalization;	
};


} // namespace smlearning

#endif // SMLEARNING_ACTIVEGNGSCENARIO_H_ 

