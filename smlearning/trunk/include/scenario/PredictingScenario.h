/** @file PredictingScenario.h
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */
#ifndef SMLEARNING_PREDICTINGSCENARIO_H_
#define SMLEARNING_PREDICTINGSCENARIO_H_

#include <scenario/Scenario.h>
#include "ssm/ssm_file_export.hh"
#include "ssm/ssm_parser.hh"
#include "cryssmex/exceptions.hh"
// #include "quantizing/quantizer_file_export.hh"

using namespace ssm;
using namespace cryssmex;
using namespace basic_stochastics;
using namespace naming;
// using namespace quantizing;

namespace smlearning {

//! \class PredictingScenario
/*! \brief Intended to be used for predicting by using Substochastic sequential machines
 */
class PredictingScenario : public Scenario {
public:
	/** Object description */
	class Desc : public Scenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PredictingScenario, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			Scenario::Desc::setToDefault();
		}
	};

	/** Objects can be constructed only in the Scene context. */
	PredictingScenario(golem::Scene &scene);
	/** destructor */
	~PredictingScenario ();
	/** get class name */
	static string getName () { return "PredictingScenario"; }
	/** Run experiment */
	void run(int argc, char* argv[]);
	/** set experiment default values */
	virtual void init(boost::program_options::variables_map vm);


protected:
	/** method for feature selection (basis/markov/...?) */
	unsigned int featureSelectionMethod;
	/** Substochastic sequential machine used for prediction */
	SSM* ssm;
	/** SSM parser used for prediction */
	SSM_Parser *ssm_parser;

};




} // namespace smlearning

#endif // SMLEARNING_PREDICTINGSCENARIO_H_ 
