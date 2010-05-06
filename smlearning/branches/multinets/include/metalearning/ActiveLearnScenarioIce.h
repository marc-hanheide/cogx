/** @file ScenarioIce.h
 * 
 * Learning scenario where the arm moves along a straight line
 * using reactive trajectory planner and collision detection
 * simulating a pushing action on a polyflap
 * 
 * Program can be run in two modes:
 * - the first uses real Katana arm
 * - the second runs the arm simulators
 *
 * offline and active modes of learning are available
 * Ice Interface
 *
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI

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

#ifndef SMLEARNING_ACTIVELEARNSCENARIOICE_H_
#define SMLEARNING_ACTIVELEARNSCENARIOICE_H_

#include <metalearning/ScenarioIce.h>
#include <metalearning/ActiveRNN.h>
#include <Helpers.hpp>
#include <PlotApp.hh>

//#include <qapplication.h>
//#include <qmainwindow.h>

//#include <tools/data_plot_test.h>

namespace smlearning {

class ActiveLearnScenarioIce : public ScenarioIce {
	ActiveRNN learner;
	//int argc;
	//char** argv;
	//Ice::ObjectPrx base;
	//TinyPrx pTiny;
public:
	///
	///constructor
	///
	ActiveLearnScenarioIce () {}

	///
	///The experiment performed in this method behaves as follows:
	///The arm selects an action according to a learning progress
	///associated to the RNN machine.
	///
	int run (int argc, char *argv[]);

	///
	///get final predicted polyflap pose from the neural network activation
	///
	golem::tinyice::Mat34 getPfPoseFromOutputActivations (rnnlib::SeqBuffer<double> outputActivations, int startIndex, Real maxRange);
	
};

}; /* namespace smlearning */

#endif
