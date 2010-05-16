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
#pragma once
#ifndef SMLEARNING_SMREGION_H_
#define SMLEARNING_SMREGION_H_

#include <vector>
#include <tools/data_handling.h>
#include <metalearning/ActiveRNN.h>

using namespace std;

namespace smlearning {

/// SMRegion class. This class encapsulates the numerical limits of
/// sensorimotor contexts defined by creating regions in a space by
/// using a statistical measure, i.e., variance.
/// It also handles the error and learning progress history of the
/// corresponding learner associated to it
struct SMRegion {

	/** index coming from a map of regions in the Scenario */
	int index;
	/** vector of minimum values of the motor context vectors in the dataset */
	vector<double> minValuesSMVector;
	/** vector of maximum values of the motor context vectors in the dataset */
	vector<double> maxValuesSMVector;
	/** size of the motor context */
	int sMContextSize;
	/** instances corresponding to the region */
	DataSet data;
	/** RNN learner corresponding to the region */
	ActiveRNN learner;
	/** vector corresponding to learning progress */
	vector<double> learningProgressHistory;
	/** vector corresponding to history of errors */
	vector<double> errorsHistory;
	/** constant to define the smoothing parameter in the evaluation of learning progress */
	static const int smoothing = 25;
	/** constant to define the time window parameter in the evaluation of learning progress */
	static const int timewindow = 15;

	SMRegion () {
	}
	
	SMRegion (const SMRegion& smRegion) :
		index(smRegion.index),
		minValuesSMVector (smRegion.minValuesSMVector),
		maxValuesSMVector (smRegion.maxValuesSMVector),
		sMContextSize (smRegion.sMContextSize),
		data (smRegion.data),
		learningProgressHistory (smRegion.learningProgressHistory),
		errorsHistory (smRegion.errorsHistory),
		learner (smRegion.learner)
	{
	}
	
	SMRegion (int idx, int smCtxtSize) :
		index(idx), sMContextSize (smCtxtSize) {
		minValuesSMVector.resize(sMContextSize, -1.0);
		maxValuesSMVector.resize(sMContextSize, 1.0);
		//learningProgressHistory.push_back (0.0);
	}

	SMRegion (SMRegion parentRegion, int idx, double cuttingValue, int cuttingIdx, DataSet inheritedData, bool firstRegion ) :
		index (idx),
		sMContextSize (parentRegion.sMContextSize),
		minValuesSMVector (parentRegion.minValuesSMVector),
		maxValuesSMVector (parentRegion.maxValuesSMVector),
		learner (parentRegion.learner),
		data (inheritedData),
		learningProgressHistory (parentRegion.learningProgressHistory),
		errorsHistory (parentRegion.errorsHistory) {
		
		//update cutting values
		//instances must be added using add_DataSet
		//method splittingCriterion2 takes care of appropriate splitting and cutting values calculation
		// for (int i=0; i<sMContextSize; i++) {
		// 	minValuesSMVector[i] = parentRegion.minValuesSMVector[i];
		// 	maxValuesSMVector[i] = parentRegion.maxValuesSMVector[i];
		// }
		
	
		if (firstRegion)
			maxValuesSMVector[cuttingIdx] = cuttingValue;
		else
			minValuesSMVector[cuttingIdx] = cuttingValue;
	}

	~SMRegion () {
	}

	///
	///update the learning progress associated to the region
	///
	void update_learning_progress (const rnnlib::DataSequence& seq);

	///
	///save region limits on a file
	///
	bool write_data (string fileName);


};


}; /* namespace smlearning */

#endif
