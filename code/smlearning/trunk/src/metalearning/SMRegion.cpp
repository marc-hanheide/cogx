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

#include <metalearning/SMRegion.h>

namespace smlearning {

///
///update the learning progress associated to the region
///
void SMRegion::updateLearnProgress (const rnnlib::DataSequence& seq) {

	errorsHistory.push_back (learner.update(seq));
	
	double timewindowRatio = timewindow / double(smoothing + timewindow);
	double smoothingRatio = smoothing / double(smoothing + timewindow);
	double smoothing = errorsHistory.size() * smoothingRatio + 1;
	int lastPrevSmoothErrorIdx = (int)ceil(errorsHistory.size() * (1 - timewindowRatio));
	double accPrevSmoothError = 0.0;
	for (int i=0; i != lastPrevSmoothErrorIdx; i++)
		accPrevSmoothError += errorsHistory[i];
	accPrevSmoothError /= smoothing;

	int firstCurrSmoothErrorIdx = (int)floor(errorsHistory.size() * (1 - smoothingRatio));
	double accCurrSmoothError = 0.0;
	for (int i=firstCurrSmoothErrorIdx; i<errorsHistory.size(); i++)
		accCurrSmoothError += errorsHistory[i];
	accCurrSmoothError /= smoothing;
	
	
	learningProgressHistory.push_back (-(accCurrSmoothError - accPrevSmoothError));
	
	cout << "\tLearning progress: " << endl;
	cout << "\t" << learningProgressHistory.back() << endl;
	
}



}; /* namespace smlearning */
