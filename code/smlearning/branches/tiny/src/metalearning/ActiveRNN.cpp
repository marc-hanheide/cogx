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

#include <metalearning/ActiveRNN.h>

namespace smlearning {

///
///construct RNN for active learning
///
void ActiveRNN::build (string dataFile, int smregionsCount, int patternSize, ostream& out) {

	dataFile += ".nc";
	header = new rnnlib::DataHeader ( dataFile, task, 1);
	net = new rnnlib::MultilayerNet(out, conf, *header);
	//build weight container after net is created
	rnnlib::WeightContainer::instance().build();
	int numWeights = rnnlib::WeightContainer::instance().weights.size();
	
	//build the network after the weight container
	net->build();
	
	//only construct optimiser after weight container is built
	if (conf.get<string>("optimiser", "steepest") == "rprop")
	{
		opt = new rnnlib::Rprop(out);
	}
	else
	{
		opt = new rnnlib::SteepestDescent(out, conf.get<double>("learnRate", 1e-4), conf.get<double>("momentum", 0.9));
	}
	out << "setting random seed to " << Random::set_seed(conf.get<unsigned long int>("randSeed", 0)) << endl << endl;
	double initWeightRange = conf.get<double>("initWeightRange", 0.1);
	out << "randomising uninitialised weights with mean 0 std. dev. " << initWeightRange << endl << endl;
	rnnlib::WeightContainer::instance().randomise(initWeightRange);	
	out << "optimiser:" << endl << *opt << endl;

	print_net_data();

	//build the map for learning progress and errors list associations
	for (int i=0; i<smregionsCount; i++) {
		vector<double> errorsHistory;
		pair<double, vector<double> > learnProg_errors;
		learnProg_errors.first = 0.0;
		learnProg_errors.second = errorsHistory;
		learnProg_errorsMap[i] = learnProg_errors;
	}
	normalizationFactor = patternSize * 4 * 0.5;

}

///
///update the machine state with current sequence
///
double ActiveRNN::update (const rnnlib::DataSequence& seq, int smregionIdx, ostream& out) {
	
	if (rnnlib::GlobalVariables::instance().isVerbose()) {
		out << "data sequence:" << endl;
		out << seq;
	}
	double error;
	for (int i=0; i<10; i++) {
		error = net->train(seq);
		opt->update_weights();
		rnnlib::WeightContainer::instance().reset_derivs();
	}
	out << "Region: " << smregionIdx << endl;
	error /= normalizationFactor;
	out << "\tError: " << endl;
	out << "\t" << error << endl;
	learnProg_errorsMap[smregionIdx].second.push_back (error);
	if (learnProg_errorsMap[smregionIdx].second.size () > SMOOTHING+TIMEWINDOW)
		learnProg_errorsMap[smregionIdx].second.erase (learnProg_errorsMap[smregionIdx].second.begin());
	out << "\tLearning progress: " << endl;
	out << "\t" << updateLearnProgress (smregionIdx) << endl;

	return error;

}

///
///update the learning progress associated to region r
///
double ActiveRNN::updateLearnProgress (int smregionIdx) {

	double timewindowRatio = TIMEWINDOW / double(SMOOTHING + TIMEWINDOW);
	double smoothingRatio = SMOOTHING / double(SMOOTHING + TIMEWINDOW);
	double smoothing = learnProg_errorsMap[smregionIdx].second.size() * smoothingRatio + 1;
	int lastPrevSmoothErrorIdx = (int)ceil(learnProg_errorsMap[smregionIdx].second.size() * (1 - timewindowRatio));
	double accPrevSmoothError = 0.0;
	for (int i=0; i != lastPrevSmoothErrorIdx; i++)
		accPrevSmoothError += learnProg_errorsMap[smregionIdx].second[i];
	accPrevSmoothError /= smoothing;

	int firstCurrSmoothErrorIdx = (int)floor(learnProg_errorsMap[smregionIdx].second.size() * (1 - smoothingRatio));
	double accCurrSmoothError = 0.0;
	for (int i=firstCurrSmoothErrorIdx; i<learnProg_errorsMap[smregionIdx].second.size(); i++)
		accCurrSmoothError += learnProg_errorsMap[smregionIdx].second[i];
	accCurrSmoothError /= smoothing; 
	return learnProg_errorsMap[smregionIdx].first = -(accCurrSmoothError - accPrevSmoothError);
	
}

///
///active selection of samples
///
int ActiveRNN::chooseSMRegion () {
	double maxLearningProgress = -1.0;
	int regionIdx = -1;
	for (int i=0; i<learnProg_errorsMap.size(); i++)
		if (learnProg_errorsMap[i].first > maxLearningProgress) {
			regionIdx = i;
			maxLearningProgress = learnProg_errorsMap[i].first;
		}
			
	return regionIdx;
	
}



}; /* namespace smlearning */
