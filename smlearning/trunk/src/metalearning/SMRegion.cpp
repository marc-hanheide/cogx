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

#include <metalearning/SMRegion.h>

namespace smlearning {

///
///update the learning progress associated to the region
///
void SMRegion::update_learning_progress (const rnnlib::DataSequence& seq) {

	errorsHistory.push_back (learner.update(seq));
	
	double timewindowRatio = timewindow / double(smoothing + timewindow);
	double smoothingRatio = smoothing / double(smoothing + timewindow);
	//double smoothing = errorsHistory.size() * smoothingRatio + 1;

	int smoothingLast = timewindow + smoothing + 1;
	int smoothingPrev = timewindow + smoothing + 1;
	int windowidxPreviousError;
	if (errorsHistory.size () <= smoothing + timewindow) {
		windowidxPreviousError = 0;
		smoothingLast = errorsHistory.size();
	}
	else
		windowidxPreviousError = errorsHistory.size() -1 - (timewindow + smoothing);

	int windowidxLastError;
	if (errorsHistory.size () <= smoothing) {
		windowidxLastError = (int)ceil((errorsHistory.size() - 1) * timewindowRatio);
		smoothingPrev = errorsHistory.size() - windowidxLastError;
	}
	else
		windowidxLastError = errorsHistory.size() - 1 - smoothing;

	assert (windowidxPreviousError >= 0 && windowidxLastError >= 0);
	double accPrevSmoothError = 0.0;
	double accLastSmoothError = 0.0;

	bool lastErrorPassed = false;
	for (int i=errorsHistory.size() - 1; i>=windowidxPreviousError; i--) {
		accPrevSmoothError += errorsHistory[i];
		if (!lastErrorPassed)
			if (i == windowidxLastError) {
				accLastSmoothError = accPrevSmoothError;
				lastErrorPassed = true;
			}
	}
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
	write_realvector (writeFile, minValuesSMVector);
	write_realvector (writeFile, maxValuesSMVector);
	write_realvector (writeFile, learningProgressHistory);
	write_realvector (writeFile, errorsHistory);
	
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
	read_realvector (readFile, minValuesSMVector);
	read_realvector (readFile, maxValuesSMVector);
	read_realvector (readFile, learningProgressHistory);
	read_realvector (readFile, errorsHistory);
	
	readFile.close ();

	return true;
	
	//return learner.write_net_data (fileName);
}


}; /* namespace smlearning */
