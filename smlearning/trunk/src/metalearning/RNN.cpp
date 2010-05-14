/** @file RNN.cpp
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

#include <metalearning/RNN.h>

namespace smlearning {

ostream& RNN::save_config_file (ostream& out)
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

void RNN::save_net_data(string netFile, ostream& out)
{
	netFile += ".net";
	assert (netFile != "");
	ofstream fout(netFile.c_str());
	if (fout.is_open())
	{
		out << "saving to " << netFile << endl;
		conf.set<bool>("loadWeights", true);
		fout << conf << net->dataExportHandler;
	}
	else
	{
		out << "WARNING trainer unable to save to file " << netFile << endl;
	}
}

void OfflineRNN::build (ostream& out) {
	string dataFile = conf.get<string>("trainFile");
	rnnlib::DataHeader header(dataFile, task, 1);
	net = new rnnlib::MultilayerNet(out, conf, header);
}

void OfflineRNN::set_testdatafile (string fileName) {
	conf.set<string>("testFile", fileName);
}

void OfflineRNN::set_traindatafile (string fileName) {
	conf.set<string>("trainFile", fileName);
}


///
///generate config files for RNNs for offline experiments
///
bool generate_network_files_nfoldcv_set (const string defaultnetConfigFile, const string baseDataFileName, int n, string target_dir ) {

	OfflineRNN myRNN;
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
		myRNN.save_config_file (netFile);
		netFile.close();
	}

	return true;
}

}; /* namespace smlearning */
