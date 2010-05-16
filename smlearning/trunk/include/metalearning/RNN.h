/** @file RNN.h
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

#ifndef SMLEARNING_RNN_H_
#define SMLEARNING_RNN_H_

#include <MultilayerNet.hpp>

namespace smlearning {

///
///RNN class. Encapsulation of LSTM learners
///
struct RNN {

	rnnlib::Mdrnn *net;
	rnnlib::ConfigFile conf;
	string task;
	
	RNN () : conf ("/usr/local/bin/SMLearning/defaultnet.config") {
		//data loaded in from config file (default values below)
		rnnlib::GlobalVariables::instance().setVerbose (false);
		task = conf.get<string>("task");
	}

	~RNN () {
	}
	
	///
	///print network topology and learning algorithm information
	///
	void print_net_data (ostream& out = cout);

	///
	///save network topology
	///
	bool write_net_data(string netFile, ostream& out = cout);
	
	///
	///set neural network file name
	///
	void set_net_file (string fileName);

	///
	///construct RNN machine using config data
	///
	virtual void build () { };

	///
	///read config file data from a given file
	///
	void set_config_file (rnnlib::ConfigFile &configFile);

	///
	///save RNN config file to be used for offline experiments
	///
	ostream& write_config_file (ostream& out = cout);
};

///
///OfflineRNN class. Structs for generating config files for offline experiments
///
struct OfflineRNN :  RNN {

	OfflineRNN () : RNN () { }

	///
	///set test data file to be used with the RNN
	///
	void set_testdatafile (string fileName);

	///
	///set train data file to be used with the RNN
	///
	void set_traindatafile (string fileName);

	///
	///construct RNN machine using config data
	///
	virtual void build (ostream& out = cout);
	
};

///
///generate config files for RNNs for offline experiments
///
bool generate_network_files_nfoldcv_set (const string defaultnetFileName, const string baseDataFileName, int n, string targetDir );



}; /* namespace smlearning */


#endif /* SMLEARNING_RNN_H_ */
