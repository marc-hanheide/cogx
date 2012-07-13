/** @file OnlineLearningDataset.h
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 2.0 beta
 *
 */

#ifndef _SMLEARNING_ONLINELEARNINGDATASET_H_
#define _SMLEARNING_ONLINELEARNINGDATASET_H_

#include <scenario/ActiveLearnScenario.h>

namespace smlearning {

class OnlineLearningDataset : public ActiveLearnScenario
{
public:

	// /** Object description */
	// class Desc : public ActiveLearnScenario::Desc {
	// protected:
	// 	/*virtual golem::Object::Ptr create(golem::Scene& scene) const {
	// 	  OnlineLearningDataset *pObject = new OnlineLearningDataset (scene);
	// 		golem::Object::Ptr pointer(pObject);
	// 		if (!pObject->create(*this))
	// 			pointer.release();
	// 		return pointer;
	// 		}*/
	// 	CREATE_FROM_OBJECT_DESC1(OnlineLearningDataset, golem::Object::Ptr, golem::Scene&)
		
	// public:
	// 	/** Constructs description object */
	// 	Desc() {
	// 		ActiveLearnScenario::Desc::setToDefault();
	// 	}
	// };
	
	/** constructor */
	OnlineLearningDataset (golem::Scene& scene);
	/** destructor */
	~OnlineLearningDataset () {};
	//! Run experiment
	/*! \brief data are taken from a given data file. After each item,
	  the algorithm is updated
	*/
	void run (int argc, char* argv[]);
	/** Set experiment default values */
	virtual void init (boost::program_options::variables_map vm);
	/** get class name */
	static string getName () { return "OnlineLearningDataset"; }
protected:
	
};

} // namespace smlearning 

#endif /* _SMLEARNING_ONLINELEARNINGDATASET_H_ */
