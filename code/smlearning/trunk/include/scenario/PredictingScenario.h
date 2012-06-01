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
#include <boost/function.hpp>
#include <metalearning/CrySSMEx.h>

namespace smlearning {

//! \class PredictingScenario
/*! \brief For predicting by using Substochastic sequential machines
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
	/** select a random action */
	virtual void chooseAction ();
	/** calculate the start coordinates of the arm */
	virtual void calculateStartCoordinates();
	/** Describe the experiment trajectory */
	virtual void initMovement();
	/** Renders the object. */
        virtual void render();
	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** update error in output prediction */
	void updateOutputError ();
	/** enumerate output labels */
	void enumerate_labels ();
	/** Encapsulation of CrySSMEx components */
	CrySSMEx cryssmex;
	/** method for feature selection */
	unsigned int featureSelectionMethod;
	// /** current data sequence (e.g. for predicting) */
	// vector<FeatureVector> currentSeq;
	/** normalization function */
	boost::function<float (const float&, const float&, const float&)> normalization;
	/** denormalization function */
	boost::function<float (const float&, const float&, const float&)> denormalization;
	/** counter to get last index of sequence to be painted */
	int counter_sequence;
	/** predicted sequence */
	vector<FeatureVector> predictedSeq;

	/** default object bounds */
	golem::Bounds::SeqPtr objectLocalBounds;
	/** current sequence from the data set when predicting from it */
	LearningData::Chunk::Seq seqDataset;
	// /** map of label seq to output quanta used to count errors */
	// std::map<string, unsigned int> output_map;
	/** average error in output prediction */
	vector<double> avgoutputerrors;
	/** current predicted output */
	vector<string> currentPredictedOutput;



};




} // namespace smlearning

#endif // SMLEARNING_PREDICTINGSCENARIO_H_ 
