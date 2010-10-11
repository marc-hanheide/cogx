/** @file data_handling.h
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

#ifndef SMLEARNING_DATAHANDLING_H_
#define SMLEARNING_DATAHANDLING_H_

#include <metalearning/Scenario.h>

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>

#include <netcdf.h>

#include <boost/filesystem/operations.hpp>
#include <boost/regex.hpp>

#include <metalearning/RNN.h>
//#include <tools/data_structs.h>
#include <tools/math_helpers.h>

using namespace std;
using namespace boost::filesystem;
using namespace golem;

namespace smlearning {


///
///function that prints the passed argument
///
template <typename T>
void print_item (const T& elem) {
    cout << elem << " ";
}

///
///print a FeatureVector struct
///
template <typename T>
void print_featvector (const vector<T>& v) {

	for_each (v.begin(), v.end(), print_item<T>);

}

///
///print a Sequence struct
///
template <typename T>
void print_sequence (const Sequence& s) {
	typename Sequence::const_iterator i;
	cout << "{";
	for (i=s.begin(); i != s.end(); i++) {
		cout << "[";
		print_featvector<T> (*i);
		if (i+1 == s.end())
			cout << "]";
		else
			cout << "], " << endl;
	}
	cout << "}\n" << endl;


}

///
///print a DataSet struct
///
template <typename T>
void print_dataset (const DataSet& d) {
	cout << "Dataset size: " << d.size() << endl;
	for_each (d.begin(), d.end(), print_sequence<T>);
}


///
///write a vector to a file
///
template <typename T>
void write_vector (ofstream& writeFile, const vector<T>& v) {
	long featvectorSize = v.size();
	writeFile.write ((const char*)&featvectorSize, sizeof (v.size()));
	typename vector<T>::const_iterator n;
	for (n=v.begin(); n!= v.end(); n++) {
		writeFile.write ((const char* )&(*n), sizeof (*n));
	}
}



///
///read a vector to a file
///
template <typename T>
void read_vector (ifstream& readFile, vector<T>& v) {
	long featvectorSize;
	readFile.read ((char *)&featvectorSize, sizeof(featvectorSize));
	// cout << "\t\t" << featvectorSize << endl;
	for (int n=0; n<featvectorSize; n++) {
		T value;
		readFile.read ((char* )&value, sizeof(value));
		v.push_back (value);
	}
}




///
///print DataSetParams tuple
///
void print_dataset_params (const DataSetParams& p);

///
///write DataSet vector to a file
///
bool write_dataset (string fileName, const DataSetStruct& data);

///
///read DataSet vector from a file
///
bool read_dataset (string fileName, DataSetStruct& data);

///
///write DataSet vector to a cdl (netcdf in text format) file using zero-padding (deprecated)
///
bool write_cdl_file_padding (string fileName, const DataSet& data);

///
///write DataSet vector to a cdl (netcdf in text format) file using basis feature vectors
///(deprecated - do not use)
bool write_cdl_file_basis (string fileName, const DataSet& data);

///
///write DataSet vector to a nc (netcdf format) file using basis feature vectors
///
bool write_nc_file_basis (string fileName, const DataSetStruct& data);

///
///write DataSet vector to a nc (netcdf format) file using a markovian based representation
///(deprecated)
bool write_nc_file_Markov (string fileName, const DataSet& data);

///
///concatenate .seq files in one file
///
bool concatenate_datasets (string dir, string writeFileName);

///
///load a sequence into inputs and target vectors (deprecated)
///
void load_sequence_Markov (vector<float>& inputVector, vector<float>& targetVector, Sequence s);

///
///load a sequence into inputs and target vectors (for active LSTM learning)
///
void load_sequence_basis (vector<float>& inputVector, vector<float>& targetVector, Sequence s, int inputSize, int outputSize, int motorVectorSize, int targetIndexStart = 0);

///
///load training data in RNNLIB format
///
rnnlib::DataSequence* load_trainSeq (smlearning::Sequence& seq, DataSetParams params);


///
///generation of n-fold cross-validation sets from a particular sequence file
///and using a certain write_netcdf_file function
///
template<class Function>
bool write_n_fold_cross_valid_sets (string seqFileName, int n, Function write_netcdf_file, string target_dir, bool print_data = false) {

	DataSetStruct data;
	if (n < 2) {
		cout << "You have to use at least 2 cross-validation sets" << endl;
		return false;
	}
	if (!read_dataset (seqFileName, data)){
		cout << "file " + seqFileName + " could not be read" << endl;
		return false;
	}

	boost::regex seqfile_re (".*/(.*)$");
	boost::cmatch match;
	string seqBaseFileName;
	if (boost::regex_match(seqFileName.c_str(), match, seqfile_re)) {
		seqBaseFileName = string(match[1].first, match[1].second);
	}
	else
		seqBaseFileName = seqFileName;
	cout << "seqFileBaseName: " << seqBaseFileName << endl;
	cout << "seqFile: " << seqFileName << endl;

	
	vector<DataSet> partitions_testing;
	vector<DataSet> partitions_training;
	
	// initialize random seed:
	srand ((unsigned)time(NULL) );
	
	// generate random number:
	int randNr;

	long int partitionSize = data.first.size() / n;
	vector<bool> availablePartitions;
	
	for (int i=0; i<n; i++) {
		DataSet partition_testing;
		DataSet partition_training;
		partitions_testing.push_back (partition_testing);
		partitions_training.push_back (partition_training);
		availablePartitions.push_back (true);		
	}

	DataSet::const_iterator s;
	for (s=data.first.begin(); s!=data.first.end(); s++) {
		bool available_partitions = false;
		//check available partitions
		for (int i=0; i<n; i++)
			if (availablePartitions[i] == true) {
				available_partitions = true;
				break;
			}

		if (available_partitions) {
			bool available_partition_found = false;
			do {
				randNr = rand() % n;
				if (availablePartitions[randNr]) {
					partitions_testing[randNr].push_back(*s);
					available_partition_found = true;
					if (partitions_testing[randNr].size() == partitionSize)
						availablePartitions[randNr] = false;
				}				
			} while (!available_partition_found);
		}
		else
			break;
	}

	//store partitions in cdl files
	for (int i=0; i< n; i++) {
		cout << "size of testing partition " << i << ": " << partitions_testing[i].size() << endl;
		if (print_data)
			print_dataset<double>(partitions_testing[i]);
		stringstream testingFileName;
		testingFileName << target_dir << "/" << seqBaseFileName << "_" << n << "_foldcv_set-" << i << "_testing";
		write_netcdf_file (testingFileName.str(), make_pair(partitions_testing[i],data.second));
		for (int j=0; j<n; j++)
			if (i != j) {
				DataSet::const_iterator s;
				for (s=partitions_testing[j].begin(); s!=partitions_testing[j].end(); s++)
					partitions_training[i].push_back (*s);
			}
		cout << "size of training partition " << i << ": " << partitions_training[i].size() << endl;
		if (print_data)
			print_dataset<double>(partitions_training[i]);
		stringstream trainingFileName;
		trainingFileName << target_dir << "/" << seqBaseFileName << "_" << n << "_foldcv_set-" << i << "_training";
		write_netcdf_file (trainingFileName.str(), make_pair(partitions_training[i], data.second));
		
	}

	return true;
	
}


///
///returns file name based on current time
///
string get_base_filename_from_time ();

///
///a utility function to obtain the file name from a possibly large path/file pattern
///
string get_seqBaseFileName (string seqFile);

///
///obtain a discretization of starting finger poses from a canonical set of actions
///
map<Vec3, int, compare_Vec3> get_canonical_positions ();

///
///enumerate a dataset using a canonical representation taking into account time steps
///
CanonicalData::DataSetStruct canonical_input_output_enumerator_with_time (DataSetStruct& data, Scenario::Desc& scenario, int modulo = 1);

///
///write a dataset in cryssmex format by using the canonical representation
///
void write_canonical_dataset_cryssmex_fmt (string writeFileName, CanonicalData::DataSet data);

///
///write a dataset in cryssmex format. This code assumes vectorial data format and an output
///label defined artificially
///
void write_dataset_cryssmex_fmt_with_label (string writeFileName, DataSet data, int modulo = 1);

///
///write a dataset in cryssmex format. This code assumes vectorial data format
///and works as a regression predictor and canonical format for the input
///\params pfefState and pfefOutput allow features with both polyflap and effector vectors
///
void write_canonical_dataset_cryssmex_fmt_regression (string writeFileName, CanonicalData::DataSetStruct& data, bool pfefState = true, bool pfefOutput = false);

///
///write a dataset in cryssmex format. This code assumes vectorial data format
///and works as a regression predictor
///\params pfefState and pfefOutput allow features with both polyflap and effector vectors
///
void write_dataset_cryssmex_fmt_regression (string writeFileName, CanonicalData::DataSetStruct& data, bool pfefState = true, bool pfefOutput = false);

///
///parse a string containing a list of starting positions (TODO: write a nice reg.exp. :P )
///
vector<int> parse_startingPositions(string str, int maxStartPos);

}; /* smlearning namespace */

#endif /* SMLEARNING_DATAHANDLING_H_*/
