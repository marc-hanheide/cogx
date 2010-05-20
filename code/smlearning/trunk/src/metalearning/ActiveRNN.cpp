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
///update the machine state with current sequence
///
double ActiveRNN::update (const rnnlib::DataSequence& seq, ostream& out) {
	
	if (rnnlib::GlobalVariables::instance().isVerbose()) {
		out << "data sequence:" << endl;
		out << seq;
	}
	double error;
	for (int i=0; i<1/*00*/; i++) {
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
	//double normalizationFactor = seq.num_timesteps() * 0.5;
	//error /= normalizationFactor;
	out << "\tError: " << endl;
	out << "\t" << error << endl;

	return error;

}

void ActiveRNN::feed_forward (const rnnlib::DataSequence& seq) {
	net->feed_forward(seq);
}




}; /* namespace smlearning */
