/** @file PredictingActiveLearnScenario.h
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */

#ifndef _SMLEARNING_PREDICTINGACTIVELEARNSCENARIO_H_
#define _SMLEARNING_PREDICTINGACTIVELEARNSCENARIO_H_

//------------------------------------------------------------------------------

#include <scenario/Scenario.h>
#include <metalearning/GNGSMRegion.h>
#include <boost/function.hpp>


//------------------------------------------------------------------------------

namespace smlearning {

//! \class PredictingActiveLearnScenario
/*! \brief For predicting by using Substochastic sequential machines learned
 *  by the procedures in \p ActiveLearnScenario
 */
class PredictingActiveLearnScenario : public Scenario {
public:
	/** Object description */
	class Desc : public Scenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PredictingActiveLearnScenario, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			Scenario::Desc::setToDefault();
		}
	};

	/** Objects can be constructed only in the Scene context. */
	PredictingActiveLearnScenario(golem::Scene &scene);
	/** destructor */
	~PredictingActiveLearnScenario ();
	/** get class name */
	static string getName () { return "PredictingActiveLearnScenario"; }
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
	/** update average error in prediction */
	void updateAvgError ();
	/** update error in output prediction */
	void updateOutputError ();
	/** enumerate output labels */
	void enumerate_labels ();
	/** normalization function */
	boost::function<float (const float&, const float&, const float&)> normalization;
	/** denormalization function */
	boost::function<float (const float&, const float&, const float&)> denormalization;
	/** map of indexed sensorimotor regions */
	GNGSMRegion::RegionsMap regions;
	/** current context Region */
	GNGSMRegion* currentRegion;
	/** method for feature selection */
	unsigned int featureSelectionMethod;
	/** predicted sequence */
	vector<FeatureVector> predictedSeq;
	/** default object bounds */
	golem::Bounds::SeqPtr objectLocalBounds;
	/** current sequence from the data set when predicting from it */
	LearningData::Chunk::Seq seqDataset;
	/** current chosen Action */
	Action chosenAction;
	/** average error in prediction */
	vector<double> avgerrors;
	/** average error in output prediction */
	vector<double> avgoutputerrors;
	/** average error in grob classification */
	vector<double> avgclassiferrors;
	/** current predicted output */
	vector<string> currentPredictedOutput;

}; // class

} // namespace

#endif /* _SMLEARNING_PREDICTINGACTIVELEARNSCENARIO_H_ */
