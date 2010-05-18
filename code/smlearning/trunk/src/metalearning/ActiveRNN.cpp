/** @file ActiveRNN.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa 
                       Alex Graves
		       
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
///initialize RNN for active learning
///
void ActiveRNN::init (int inputPatternSize, int targetPatternSize, string netconfigFileName, ostream& out) {

	header = new rnnlib::DataHeader ( inputPatternSize, targetPatternSize);

	if (netconfigFileName.empty())
		net = new rnnlib::MultilayerNet(out, conf, *header);
	else {
		rnnlib::ConfigFile config (netconfigFileName);
		set_config_file (config);
		net = new rnnlib::MultilayerNet(out, conf, *header);
	}
	//build weight container after net is created
	net->weightContainer.build();

	build (out);
}

///
///initialize RNN for active learning
///
void ActiveRNN::init (int inputPatternSize, int targetPatternSize, rnnlib::WeightContainer& wC, ostream& out) {

	header = new rnnlib::DataHeader ( inputPatternSize, targetPatternSize);
	net = new rnnlib::MultilayerNet(out, conf, *header);
	//build weight container after net is created
	net->weightContainer.build();
	net->weightContainer.weights = wC.weights;
	net->weightContainer.derivatives = wC.derivatives;
	net->weightContainer.plasticities = wC.plasticities;
	net->weightContainer.connections = wC.connections;

	conf.set<bool>("loadWeights", false);

	build (out);
}


///
///construct RNN for active learning
///
/*void ActiveRNN::build (ostream& out) {
	
	int numWeights = net->weightContainer.weights.size();
	out << numWeights << " weights" << endl << endl;

	//build the network after the weight container
	net->build();
	
	//only construct optimiser after weight container is built
	if (conf.get<string>("optimiser", "steepest") == "rprop")
	{
		opt = new rnnlib::Rprop(out, &(net->weightContainer), &(net->dataExportHandler));
	}
	else
	{
		opt = new rnnlib::SteepestDescent(out, &(net->weightContainer), &(net->dataExportHandler), conf.get<double>("learnRate", 1e-4), conf.get<double>("momentum", 0.9));
	}
	out << "setting random seed to " << Random::set_seed(conf.get<unsigned long int>("randSeed", 0)) << endl << endl;

	if (conf.get<bool>("loadWeights", false))
	{
		out << "loading dynamic data from "  << conf.filename << endl;
		net->dataExportHandler.load(conf, out);
		// out << "epoch = " << trainer.epoch << endl << endl;
	}
	
	double initWeightRange = conf.get<double>("initWeightRange", 0.1);
	out << "randomising uninitialised weights with mean 0 std. dev. " << initWeightRange << endl << endl;
	net->weightContainer.randomise(initWeightRange);
	out << "optimiser:" << endl << *opt << endl;

	print_net_data();


}*/

///
///update the machine state with current sequence
///
double ActiveRNN::update (const rnnlib::DataSequence& seq, /*int smregionIdx,*/ ostream& out) {
	
	if (rnnlib::GlobalVariables::instance().isVerbose()) {
		out << "data sequence:" << endl;
		out << seq;
	}
	double error;
	for (int i=0; i<100; i++) {
// 	int i = 0;
// 	do {
		error = net->train(seq);
// 		out << "\tError: " << endl;
// 		out << "\t" << error << endl;
		opt->update_weights();
		net->weightContainer.reset_derivs();
// 		i++;
	}
// 	} while (error > 10.0 || i < 10);
	double normalizationFactor = seq.num_timesteps() * 0.5;
	error /= normalizationFactor;
	out << "\tError: " << endl;
	out << "\t" << error << endl;

	return error;

}

void ActiveRNN::feed_forward (const rnnlib::DataSequence& seq) {
	net->feed_forward(seq);
}




}; /* namespace smlearning */
