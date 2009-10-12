/** @file ActiveRNN.h
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

#ifndef SMLEARNING_ACTIVERNN_H_
#define SMLEARNING_ACTIVERNN_H_

#include <metalearning/RNN.h>

#include <NetcdfDataset.hpp>
#include <WeightContainer.hpp>
#include <SteepestDescent.hpp>
#include <Rprop.hpp>

#include <map>

namespace smlearning {


///
///encapsulation of structs to generate RNNs in an active learning context
///
struct ActiveRNN : RNN {

	ActiveRNN () : RNN () {	}
	
	///
	///associative map of an indexed sensorimotor region and corresponding
	///learning progress and error history
	///
	map<int, pair<vector<double>, double> > learnProg_errorsMap;

	///methods

	///
	///construct RNN machine using config data
	///
	virtual void build (const string& dataFile, ostream& out = cout);
	
	///update the machine state with current sequence
	///
	double update (const rnnlib::DataSequence& seq, ostream& out = cout);
};

}; /* namespace smlearning */

#endif
