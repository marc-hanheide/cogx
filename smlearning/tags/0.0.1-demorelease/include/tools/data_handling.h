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

#include <cstdlib>
#include <ctime>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>

#include <netcdf.h>

#include <boost/filesystem/operations.hpp>
#include <boost/regex.hpp>

using namespace std;
using namespace boost::filesystem;

namespace smlearning {

typedef vector<double> FeatureVector;
typedef vector<FeatureVector> Sequence;
typedef vector<Sequence> DataSet;

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
	for_each (d.begin(), d.end(), print_sequence<T>);
}

///
///generation of random sequences (for testing purposes)
///
void generate_rand_sequences (DataSet& data, long numSeq, long seqSize);

///
///write DataSet struct to a file
///
bool write_dataset (string fileName, const DataSet& data);

///
///read DataSet struct from a file
///
bool read_dataset (string fileName, DataSet& data);

///
///write DataSet struct to a cdl (netcdf in text format) file using zero-padding
///
bool write_cdl_file_padding (string fileName, const DataSet& data);

///
///write DataSet struct to a cdl (netcdf in text format) file using basis feature vectors
///
bool write_cdl_file_basis (string fileName, const DataSet& data);

///
///write DataSet struct to a nc (netcdf format) file using basis feature vectors
///
bool write_nc_file_basis (string fileName, const DataSet& data);

///
///concatenate .seq files in one file
///
bool concatenate_datasets (string dir, string writeFileName);

///
///load a sequence into inputs and target vectors (for machine learning)
///
void load_sequence (vector<float>& inputVector, vector<float>& targetVector, Sequence s);

///
///generation of n-fold cross-validation sets from a particular sequence file
///and using a certain write_netcdf_file function
template<class Function>
bool write_n_fold_cross_valid_sets (string seqFileName, int n, Function write_netcdf_file, string target_dir, bool print_data = false) {

	DataSet data;
	if (n < 2)
		return false;
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

	long int partitionSize = data.size() / n;
	vector<bool> availablePartitions;
	
	for (int i=0; i<n; i++) {
		DataSet partition_testing;
		DataSet partition_training;
		partitions_testing.push_back (partition_testing);
		partitions_training.push_back (partition_training);
		availablePartitions.push_back (true);		
	}

	DataSet::const_iterator s;
	for (s=data.begin(); s!=data.end(); s++) {
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
		write_netcdf_file (testingFileName.str(), partitions_testing[i]);
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
		write_netcdf_file (trainingFileName.str(), partitions_training[i]);
		
	}

	return true;
	
}


///
///write collected data in an offline experiment
///
void writeDownCollectedData(DataSet data);


}; /* smlearning namespace */

#endif /* SMLEARNING_DATAHANDLING_H_*/
