/** @file SMRegion.cpp
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

#include <tools/data_handling.h>
#include <metalearning/SMRegion.h>

namespace smlearning {

///
///update the learning progress associated to the region
///
void SMRegion::update_learning_progress (const rnnlib::DataSequence& seq) {

	errorsHistory.push_back (learner.update(seq));
	
	double timewindowRatio = timewindow / double(smoothing + timewindow);
	double smoothingRatio = smoothing / double(smoothing + timewindow);

	int smoothingLast = smoothing + 1;
	int smoothingPrev = smoothing + 1;

	int windowfirstidxPreviousError;
	int windowlastidxPreviousError;
	int windowfirstidxLastError;
	if (errorsHistory.size () <= smoothing + timewindow) {
		windowfirstidxPreviousError = 0;
	}
	else
		windowfirstidxPreviousError = errorsHistory.size() -1 - (timewindow + smoothing);

	if (errorsHistory.size () <= timewindow)
		windowlastidxPreviousError = (int)ceil((errorsHistory.size() - 1) * smoothingRatio);
	else
		windowlastidxPreviousError = errorsHistory.size() - 1 - timewindow;

	
	if (errorsHistory.size () <= smoothing) {
		windowfirstidxLastError = (int)ceil((errorsHistory.size() - 1) * timewindowRatio);
		smoothingPrev = errorsHistory.size() - windowfirstidxLastError;
		smoothingLast = errorsHistory.size() - windowfirstidxLastError;
	}
	else
		windowfirstidxLastError = errorsHistory.size() - 1 - smoothing;

	assert (windowfirstidxPreviousError >= 0 && windowlastidxPreviousError >= 0 && windowfirstidxLastError >= 0);
	double accPrevSmoothError = 0.0;
	double accLastSmoothError = 0.0;

	
	for (int i=windowfirstidxLastError; i < errorsHistory.size(); i++) 
		accLastSmoothError += errorsHistory[i];
	for (int i=windowfirstidxPreviousError; i<windowlastidxPreviousError; i++)
		accPrevSmoothError += errorsHistory[i];	
	
	accPrevSmoothError /= smoothingPrev;
	accLastSmoothError /= smoothingLast;

	learningProgressHistory.push_back (-(accLastSmoothError - accPrevSmoothError));

	
	cout << "\tLearning progress: " << endl;
	cout << "\t" << learningProgressHistory.back() << endl;
	
}

///
///save region limits and learner data into a file
///
bool SMRegion::write_data (string fileName) {
	string regFileName = fileName + ".reg";
	ofstream writeFile (regFileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;  

	writeFile.write ((const char*)&index, sizeof (index));
	write_vector<double> (writeFile, minValuesSMVector);
	write_vector<double> (writeFile, maxValuesSMVector);
	write_vector<double> (writeFile, learningProgressHistory);
	write_vector<double> (writeFile, errorsHistory);
	write_vector<double> (writeFile, startingPositionsHistory);
	


	writeFile.close ();

	return learner.write_net_data (fileName);
}

///
///read region limits and learner data from a file
///
bool SMRegion::read_data (string fileName) {
	string regFileName = fileName + ".reg";
	ifstream readFile (regFileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;  

	readFile.read ((char *)&index, sizeof(index));
	read_vector<double> (readFile, minValuesSMVector);
	read_vector<double> (readFile, maxValuesSMVector);
	read_vector<double> (readFile, learningProgressHistory);
	read_vector<double> (readFile, errorsHistory);
	read_vector<double> (readFile, startingPositionsHistory);
	
	
	readFile.close ();

	learner.init (LearningData::motorVectorSize + LearningData::efVectorSize + LearningData::pfVectorSize, LearningData::pfVectorSize, fileName + ".net");

	return true;
}

void SMRegion::print_data () {
	cout << "minValuesSMVector: " << endl;
	print_featvector<double> (minValuesSMVector);
	cout << endl << "maxValuesSMVector: " << endl;
	print_featvector<double> (maxValuesSMVector);
	cout << endl << "learningProgressHistory: " << endl;
	print_featvector<double> (learningProgressHistory);
	cout << endl << "errorsHistory: " << endl;
	print_featvector<double> (errorsHistory);
	cout << endl << "startingPositionsHistory: " << endl;
	print_featvector<double> (startingPositionsHistory);
	cout << endl;
}


///
///Find the appropriate region index according to the given sensorimotor context
///
int SMRegion::get_SMRegion (const SMRegion::RegionsMap& regions, const FeatureVector& sMContext) {
	for (RegionsMap::const_iterator regionIter = regions.begin(); regionIter != regions.end(); regionIter++) {
		bool wrongCuttingValue = false;
		SMRegion currentRegion = regionIter->second;
		//assert (sMContext.size() == currentRegion.sMContextSize);
		// cout << "sMContext size: " << sMContext.size() << endl;
		// cout << "motorVectorsize: " << SMRegion::motorVectorSize << endl;
		assert (sMContext.size() == LearningData::motorVectorSize);
		//for (int i=0; i<currentRegion.sMContextSize; i++) {
		for (int i=0; i<LearningData::motorVectorSize; i++) {
			if ( currentRegion.minValuesSMVector[i] > sMContext[i] ||
			     currentRegion.maxValuesSMVector[i] < sMContext[i]) {
				wrongCuttingValue = true;
				break;
			}
		}
		if (wrongCuttingValue) continue; else return currentRegion.index;
	}
	return -1;
}



}; /* namespace smlearning */
