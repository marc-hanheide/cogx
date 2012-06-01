/** @file GNGSMRegion.h
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
#ifndef SMLEARNING_GNGSMREGION_H_
#define SMLEARNING_GNGSMREGION_H_

#include <vector>
#include <metalearning/CrySSMEx.h>
#include <boost/unordered_map.hpp>

// using namespace std;

namespace smlearning {

//! \class GNGSMRegion
/*! \brief This class encapsulates the numerical limits of
  sensorimotor contexts defined by creating regions in a space by
  using a statistical measure, i.e., variance.
  It also handles the error and learning progress history of the
  corresponding learner associated to it
*/
struct GNGSMRegion {

	typedef boost::unordered_map<int, GNGSMRegion> RegionsMap;

	/** Encapsulation of input/output quantizers */
	CrySSMEx cryssmex;
	/** index coming from a map of regions in the Scenario */
	int index;
	/** vector of minimum values of the motor context vectors in the dataset */
	std::vector<double> minValuesSMVector;
	/** vector of maximum values of the motor context vectors in the dataset */
	std::vector<double> maxValuesSMVector;
	/** size of the motor context */
	int sMContextSize;
	/** instances corresponding to the region */
	LearningData::DataSet data;
	// /** vector corresponding to history of starting positions */
	// vector<double> startingPositionsHistory;	
	/** vector of history of avg errors for input quantizer */
	std::vector<double> inputqErrorsHistory;
	/** vector of history of avg errors for output quantizer */
	std::vector<double> outputqErrorsHistory;
	/** vector of history of graph size for input quantizer */
	std::vector<double> inputqGraphSizeHistory;
	/** vector of history of graph size for output quantizer */
	std::vector<double> outputqGraphSizeHistory;

	GNGSMRegion () { }
	
	GNGSMRegion (int idx, int smCtxtSize);

	GNGSMRegion (GNGSMRegion& parentRegion, int idx, double cuttingValue, int cuttingIdx, LearningData::DataSet& inheritedData, bool firstRegion);

	/** default cto. */
	~GNGSMRegion () {}

	/** get the average error of input quantizer */
	double getAvgError ();
	/** update errors history */
	void updateErrorsHistory ();
	/** update number of nodes history */
	void updateGraphSizeHistory ();
	/** enable MDL history saving */
	void saveMDLHistory ();
	/** save region limits and learner data into a file */
	bool writeData (string fileName);
	/** read region limits and learner data from a file */
	bool readData (string fileName);
	/** print region related variables */
	void printData ();
	/** redirect output of quantizers to /dev/null */
	void redirectOutputToNull ();
	/** Find the appropriate region index according to the given sensorimotor context */
	static int getSMRegion (const RegionsMap& regions, const FeatureVector& SMContext);
	/** Check if sMContext is a member of this region */
	bool checkSMRegionMembership (const FeatureVector& sMContext);
	/** Generate CVQ quantizer and SSM files from input and output quantizers */
	void generateCryssmexFiles (unsigned int max_iterations, ssm::SSM::Type ssm/*, bool save_all*/, string prefix, string regionFileName, LearningData::FeaturesLimits& limits, feature_selection featureSelectionMethod);
};


}; /* namespace smlearning */

#endif
