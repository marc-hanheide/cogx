/** @file ActiveLearnScenario.h
 * 
 * @author	Sergio Roa (DFKI)
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#ifndef _SMLEARNING_ACTIVELEARNSCENARIO_H_
#define _SMLEARNING_ACTIVELEARNSCENARIO_H_

//------------------------------------------------------------------------------

#include <scenario/Scenario.h>
#include <metalearning/GNGSMRegion.h>
#include <boost/function.hpp>


//------------------------------------------------------------------------------

namespace smlearning {


//------------------------------------------------------------------------------

/** ActiveLearnScenario class */
class ActiveLearnScenario : public smlearning::Scenario {
public:
	/** Object description */
	class Desc : public Scenario::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(ActiveLearnScenario, golem::Object::Ptr, golem::Scene&)

	public:
		/** Constructs description object */
		Desc() {
			Scenario::Desc::setToDefault();
		}
	};

	/** Objects can be constructed only in the Scene context. */
	ActiveLearnScenario(golem::Scene &scene);
	/** destructor */
	~ActiveLearnScenario () {}

	/** Run experiment */
	///
	///The experiment performed in this method behaves as follows:
	///The arm either selects an action which maximizes the learning progress
	///or selects any of the possible actions randomly according to a near greedy rule.
	///Active Learning is implemented using Intelligent Artificial Curiosity algorithm.
	///
	void run(int argc, char* argv[]);

	/** set experiment default values*/
	virtual void init(boost::program_options::variables_map vm);
	/** get class name */
	static string getName () { return "ActiveLearnScenario"; }

	
protected:

	/** Encapsulation of CrySSMEx components */
	ActiveCrySSMEx cryssmex;

	/** map of indexed sensorimotor regions */
	GNGSMRegion::RegionsMap regions;

	/** type for vector of candidate actions */
	typedef vector<Action> ActionsVector;

	/** regions counter for defining regions indices */
	int regionsCount;

	/** current context Region */
	GNGSMRegion* currentRegion;
	/** method for feature selection */
	unsigned int featureSelectionMethod;
	/** constant number of maximum candidate actions */
	static const int maxNumberCandidateActions = 1000;
	/** near greedy action selection probability bound for choosing random actions */
	static const double neargreedyActionProb = 0.3;
	/** number of instances to decide splitting a region */
	int splittingCriterion1;
	/** size of motor Context Vector */
	static const int motorContextSize = 6;
	/** size of motor Feature Vector */
	int motorVectorSize;
	/** current chosen Action from a set */
	Action *chosenAction;

	// /** canonical positions */
	// map<Vec3, int, compare_Vec3> positionsT;
	
	// /** Renders the object. */
        // virtual void render();

	/** (Post)processing function called AFTER every physics simulation step and before rendering. */
	virtual void postprocess(golem::SecTmReal elapsedTime);
	/** normalization function */
	boost::function<float (const float&, const float&, const float&)> normalization;
	/** denormalization function */
	boost::function<float (const float&, const float&, const float&)> denormalization;
	/** select an optimal action from a random set of actions */
	virtual void chooseAction ();
	/** calculate the start coordinates of the arm */
	virtual void calculateStartCoordinates();
	/** Describe the experiment trajectory */
	virtual void initMovement();
	/** write obtained dataset into a binary file */
	virtual void writeData (bool final = false);
	/** get the action corresponding to a region of high error */
	int getActionMaxAvgError (const ActionsVector& candidateActions);
	/** Update learners according to a sensorimotor region splitting criterion */
	void updateLearners (int);
	/** Obtain current context region pointer */
	void updateCurrentRegion ();
	/** variance calculation of a dataset */
	double variance (const LearningData::DataSet& data, int sMContextSize);
	/** minimality criterion for splitting a sensorimotor region */
	double evaluateMinimality (const LearningData::DataSet& firstSplittingSet, const LearningData::DataSet& secondSplittingSet, int sMContextSize);
	/** partition of regions according to variance of dataset instances */
	void splitRegion (GNGSMRegion& region);
	/** flag for saving MDL history */
	bool saveMDLHistory;

};


};	// namespace

#endif /* _SMLEARNING_ACTIVELEARNSCENARIO_H_ */
