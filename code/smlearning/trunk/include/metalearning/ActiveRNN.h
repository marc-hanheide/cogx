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
#pragma once
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

	~ActiveRNN () {
	}

	///
	///Optimization algorithm
	///
	rnnlib::Optimiser* opt;
	rnnlib::DataHeader *header;

	/*	///
	///constants to define the smoothing and time window parameters in the evaluation of learning progress
	///
	static const int SMOOTHING = 25;
	static const int TIMEWINDOW = 15;
	static const double neargreedyActionProb = 0.35; */
	///
	///normalization factor for prediction error
	///
	double normalizationFactor;

	///
	///associative map of an indexed sensorimotor region and corresponding
	///learning progress and error history
	///
	// map<int, pair<vector<double>, vector<double> > > learnProg_errorsMap;

	///methods

	///
	///initialize RNN for active learning
	///
	virtual void init (/*int smregionsCount,*/ int inputPatternSize, int targetPatternSize, ostream& out = cout);
	///
	///initialize RNN for active learning
	///
	virtual void init (int inputPatternSize, int targetPatternSize, rnnlib::WeightContainer& wC, ostream& out = cout);
	///
	///construct RNN machine using config data for active learning
	///
	virtual void build (ostream& out = cout);

	///
	///update the machine state with current sequence
	///
	double update (const rnnlib::DataSequence& seq/*, int smregionIdx*/, ostream& out = cout);


	///
	///feedforward sequence
	///
	void feed_forward (const rnnlib::DataSequence& seq);
	
	///
	///update the learning progress associated to region r
	///
	double updateLearnProgress (int smregionIdx);

	///
	///active selection of samples
	///
	int chooseSMRegion ();
	
};

}; /* namespace smlearning */

#endif
