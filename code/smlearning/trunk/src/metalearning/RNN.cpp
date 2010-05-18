/** @file RNN.cpp
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

#include <metalearning/RNN.h>

namespace smlearning {


///
///construct RNN
///
void RNN::build (ostream& out) {
	
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


}


ostream& RNN::write_config_file (ostream& out)
{
	try {
		out << conf;
	} catch (ofstream::failure e) {
		cout << "Exception opening/reading file";
	}

	return out;
}

void RNN::set_config_file (rnnlib::ConfigFile &configFile) {
	conf = configFile;
}

void RNN::print_net_data (ostream& out)
{
	out << endl << "network:" << endl;
	PRINT(task, out);
	out << *net;
}

bool RNN::write_net_data(string netFile, ostream& out)
{
	netFile += ".net";
	assert (netFile != "");
	ofstream fout(netFile.c_str());
	if (fout.is_open())
	{
		out << "saving to " << netFile << endl;
		conf.set<bool>("loadWeights", true);
		fout << conf << net->dataExportHandler;
		return true;
	}
	else
	{
		out << "WARNING trainer unable to save to file " << netFile << endl;
		return false;
	}
}

void RNN::init (ostream& out) {
	string dataFile = conf.get<string>("trainFile");
	header = new rnnlib::DataHeader (dataFile, task, 1);
	net = new rnnlib::MultilayerNet(out, conf, *header);

	//build weight container after net is created
	net->weightContainer.build();

	build (out);


}

void RNN::set_testdatafile (string fileName) {
	conf.set<string>("testFile", fileName);
}

void RNN::set_traindatafile (string fileName) {
	conf.set<string>("trainFile", fileName);
}


///
///generate config files for RNNs for offline experiments (n-fold cross-validation)
///
bool generate_network_files_nfoldcv_set (const string defaultnetConfigFile, const string baseDataFileName, int n, string target_dir ) {

	RNN myRNN;
	rnnlib::ConfigFile conf(defaultnetConfigFile);
	myRNN.set_config_file (conf);

	for (int i=0; i<n; i++) {
		stringstream testingFileName;
		stringstream trainingFileName;
		stringstream netFileName;
		netFileName << target_dir << "/" << baseDataFileName << "_" << n << "_foldcv_set-" << i << ".config";
		ofstream netFile (netFileName.str().c_str());
		testingFileName << baseDataFileName << "_" << n << "_foldcv_set-" << i << "_testing.nc";
		trainingFileName << baseDataFileName << "_" << n << "_foldcv_set-" << i << "_training.nc";
		
		myRNN.set_testdatafile (testingFileName.str());
		myRNN.set_traindatafile (trainingFileName.str());
		myRNN.write_config_file (netFile);
		netFile.close();
	}

	return true;
}


}; /* namespace smlearning */
