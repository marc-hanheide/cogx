/** @file data_structs.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa
 
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
#ifndef SMLEARNING_DATASTRUCTS_H_
#define SMLEARNING_DATASTRUCTS_H_

#include <vector>

#include <Tools/Tools.h>

#include <boost/tuple/tuple.hpp>

using namespace golem;
using namespace std;
using namespace boost;

namespace smlearning {

///
///this representation should also allow for labels, i.e., a vector of size 1
///properly discretized
///
typedef vector<double> FeatureVector;
typedef vector<FeatureVector> Sequence;
typedef vector<Sequence> DataSet;
/** LearningParams tuple <motorVectorSize, featureVectorSize, pfVectorSize, efVectorSize> */
enum { mVSize, fVSize, pfVSize, efVSize };
typedef tuple<int, int, int, int> LearningParams;
/** DataSetValueLimits tuple < minX, minY, minZ, maxX, maxY, maxZ> */
enum { minX, minY, minZ, maxX, maxY, maxZ };
typedef tuple<Real, Real, Real, Real, Real, Real> DataSetValueLimits;
/** DataSetParams tuple <LearningParams, storeLabels, DataSetValueLimits> */
enum {lParams, labels, limits };
typedef tuple<LearningParams, bool, DataSetValueLimits> DataSetParams;
typedef pair<DataSet, DataSetParams> DataSetStruct;

struct CanonicalData {
	struct FeatureVector {
		smlearning::FeatureVector rawVector;
		string motorCommand;
		string label;
	};
	typedef vector<FeatureVector> Sequence;
	typedef vector<Sequence> DataSet;
	typedef pair<DataSet, DataSetParams> DataSetStruct;
	
	DataSetStruct data;
};


/** Learning data format */
class LearningData {
public:
	/** Data chunk */
	class Chunk {
	public:
		typedef std::vector<Chunk> Seq;
		
		/** Do nothing */
		Chunk() {
		}
		
		/** Data chunk time stamp */
		golem::SecTmReal timeStamp;
		
		/** Arm state - (joint) dynamic configuration */
		golem::GenConfigspaceState armState;
		/** End-effector GLOBAL pose */
		golem::Mat34 effectorPose;
		/** Object GLOBAL pose */
		golem::Mat34 objectPose;
		/** End-effector orientation in Euler coordinates */
		golem::Real efRoll, efPitch, efYaw; 
		/** Object orientation in Euler coordinates */
		golem::Real obRoll, obPitch, obYaw; 
		
	};

	/** (Dynamic) Effector bounds in LOCAL coordinates; to obtain global pose multiply by Chunk::effectorPose */
	golem::Bounds::Seq effector;
	/** (Dynamic) Object bounds in LOCAL coordinates; to obtain global pose multiply by Chunk::objectPose */
	golem::Bounds::Seq object;
	/** (Static) Obstacles bounds in GLOBAL coordinates (usually ground plane) */
	golem::Bounds::Seq obstacles;
	
	/** Time-dependent data */
// 	Chunk::Seq data;
	DataSet data;
	/** current predicted polyflap poses sequence */
	vector<Mat34> currentPredictedPfSeq;
	/** current predicted effector poses sequence */
	vector<Mat34> currentPredictedEfSeq;
	/** current polyflap poses and motor command sequence */
	smlearning::Sequence currentSeq;
	/** current motor command */
	FeatureVector currentMotorCommandVector;
	/** Record validity */
	//bool bArmState;
	//bool bEffectorPose;
	//bool bObjectPose;
	//bool bFtsData;
	//bool bImageIndex;
	//bool bEffector;
	//bool bObject;
	//bool bObstacles;

	/** Reset to default (empty)*/
	void setToDefault() {
		effector.clear();
		object.clear();
		obstacles.clear();
		data.clear();
		//bArmState = false;
		//bEffectorPose = false;
		//bObjectPose = false;
		//bFtsData = false;
		//bImageIndex = false;
		//bEffector = false;
		//bObject = false;
		//bObstacles = false;
	}
	/** Check if the data is valid */
	bool isValid() const {
		if (!data.empty()) // must not be empty
			return false;
		//if (bEffector && effector.empty())
		//	return false;
		//if (bObject && object.empty())
		//	return false;
		//if (bObstacles && obstacles.empty())
		//	return false;

		return true;
	}
};

}; /* smlearning namespace */

#endif /* SMLEARNING_DATASTRUCTS_H_*/
