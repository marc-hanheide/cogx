/** @file SMRegion.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2010      Sergio Roa
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */
#ifndef SMLEARNING_SMREGION_H_
#define SMLEARNING_SMREGION_H_

#include <vector>
#include <metalearning/ActiveRNN.h>
#include <tools/data_structs.h>



using namespace std;

namespace smlearning {

/// SMRegion class. This class encapsulates the numerical limits of
/// sensorimotor contexts defined by creating regions in a space by
/// using a statistical measure, i.e., variance.
/// It also handles the error and learning progress history of the
/// corresponding learner associated to it


struct SMRegion {

	typedef map<int, SMRegion> RegionsMap;

	/** index coming from a map of regions in the Scenario */
	int index;
	/** vector of minimum values of the motor context vectors in the dataset */
	vector<double> minValuesSMVector;
	/** vector of maximum values of the motor context vectors in the dataset */
	vector<double> maxValuesSMVector;
	/** size of the motor context */
	int sMContextSize;
	/** instances corresponding to the region */
	// DataSet data;
	LearningData::DataSet data;
	/** RNN learner corresponding to the region */
	ActiveRNN learner;
	/** vector corresponding to learning progress */
	vector<double> learningProgressHistory;
	/** vector corresponding to history of errors */
	vector<double> errorsHistory;
	/** vector corresponding to history of starting positions */
	vector<double> startingPositionsHistory;
	/** constant to define the smoothing parameter in the evaluation of learning progress */
	int smoothing;
	/** constant to define the time window parameter in the evaluation of learning progress */
	int timewindow;
	

	SMRegion () {
	}
	
	SMRegion (int idx, int smCtxtSize, int splittingCriterion1/*, int created = 0*/) :
		index(idx),
		sMContextSize (smCtxtSize)
	{
		minValuesSMVector.resize(sMContextSize, -1.0);
		maxValuesSMVector.resize(sMContextSize, 1.0);
		// minValuesSMVector.resize(LearningData::motorVectorSize, -1.0);
		// maxValuesSMVector.resize(LearningData::motorVectorSize, 1.0);
		timewindow = splittingCriterion1 * 0.375;
		smoothing = splittingCriterion1 * 0.625;
		cout << "timewindow: " << timewindow << ", smoothing: " << smoothing << endl;
		//learningProgressHistory.push_back (0.0);
		
		
		
	}

	SMRegion (SMRegion parentRegion, int idx, double cuttingValue, int cuttingIdx, LearningData::DataSet inheritedData, bool firstRegion) :
		index (idx),
		sMContextSize (parentRegion.sMContextSize),
		minValuesSMVector (parentRegion.minValuesSMVector),
		maxValuesSMVector (parentRegion.maxValuesSMVector),
		learner (parentRegion.learner),
		data (inheritedData),
		learningProgressHistory (parentRegion.learningProgressHistory),
		errorsHistory (parentRegion.errorsHistory),
		startingPositionsHistory (parentRegion.startingPositionsHistory),
		timewindow (parentRegion.timewindow),
		smoothing (parentRegion.smoothing) {
		
		//method splittingCriterion2 in @class ActiveLearnScenario takes care of appropriate splitting and cutting values calculation

	
		if (firstRegion){
			maxValuesSMVector[cuttingIdx] = cuttingValue;
		}else {
			minValuesSMVector[cuttingIdx] = cuttingValue;
		}



	}

	~SMRegion () {
	}

	///
	///update the learning progress associated to the region
	///
	void update_learning_progress (const rnnlib::DataSequence& seq);

	///
	///save region limits and learner data into a file
	///
	bool write_data (string fileName);

	///
	///read region limits and learner data from a file
	///
	bool read_data (string fileName);

	///
	///print region related variables
	///
	void print_data ();

	///
	///Find the appropriate region index according to the given sensorimotor context
	///
	static int get_SMRegion (const RegionsMap& regions, const FeatureVector& SMContext);
	
};


}; /* namespace smlearning */

#endif
