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
///open RNN for verification
///
void ActiveRNN::build (const string& dataFile, ostream& out) {

	rnnlib::DataHeader header( dataFile, task, 1);
	net = new rnnlib::MultilayerNet(out, conf, header);
	//build weight container after net is created
	rnnlib::WeightContainer::instance().build();
	int numWeights = rnnlib::WeightContainer::instance().weights.size();
	
	//build the network after the weight container
	net->build();
	
	//only construct optimiser after weight container is built
	rnnlib::Optimiser* opt;
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

	
}

double ActiveRNN::update (const rnnlib::DataSequence& seq, ostream& out) {
	
	if (rnnlib::GlobalVariables::instance().isVerbose()) {
		out << "data sequence:" << endl;
		out << seq;
	}
	double error = net->train(seq);
	out << "error: " << endl;
	out << error << endl;
	return error;

}


}; /* namespace smlearning */
