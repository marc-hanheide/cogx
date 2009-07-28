/** @file data_handling.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2007,2008 Alex Graves
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

#ifndef TOOLS_DATAHANDLING_H_
#define TOOLS_DATAHANDLING_H_

#include <cstdlib>
#include <ctime>
#include <vector>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/framework/LocalFileInputSource.hpp>
#include <xercesc/dom/DOMException.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xercesc/sax/SAXException.hpp>
#include <xercesc/util/NameIdPool.hpp>
#include <xercesc/util/PlatformUtils.hpp>
#include <xercesc/framework/XMLValidator.hpp>
#include <xercesc/validators/schema/SchemaValidator.hpp>
#include <xercesc/validators/common/ContentSpecNode.hpp>
#include <xercesc/validators/schema/SchemaSymbols.hpp>
#include <xercesc/util/OutOfMemoryException.hpp>
#include <nn/netcdf/NetcdfDataset.h>
#include <nn/engine/NetMaker.h>
#include <nn/engine/GradientFollowerMaker.h>
#include <nn/engine/DataExportHandler.h>
#include <nn/engine/GradientTest.h>
#include <nn/random/Random.h>
#include <nn/common/Typedefs.h>
#include <nn/common/Helpers.h>

#include <netcdf.h>

#include <boost/filesystem/operations.hpp>
#include <boost/regex.hpp>

using namespace std;
using namespace boost::filesystem;

namespace smlearning {

typedef vector<double> FeatureVector;
typedef vector<FeatureVector> Sequence;
typedef vector<Sequence> DataSet;

// function that prints the passed argument
template <typename T>
void print_item (const T& elem) {
    cout << elem << " ";
}

template <typename T>
void print_featvector (const FeatureVector& v) {

	for_each (v.begin(), v.end(), print_item<T>);

}

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

template <typename T>
void print_dataset (const DataSet& d) {
	for_each (d.begin(), d.end(), print_sequence<T>);
}


void generate_rand_sequences (DataSet& data, long numSeq, long seqSize);

bool write_dataset (string fileName, const DataSet& data);

bool read_dataset (string fileName, DataSet& data);

bool write_cdl_file_padding (string fileName, const DataSet& data);

bool write_cdl_file_basis (string fileName, const DataSet& data);

bool write_nc_file_basis (string fileName, const DataSet& data);

bool concatenate_datasets (string dir, string writeFileName);

template<class Function>
bool write_n_fold_cross_valid_sets (string baseFileName, int n, Function write_netcdf_file, string target_dir, bool print_data = false) {

	DataSet data;
	if (!read_dataset (baseFileName, data))
		return false;
	
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
		testingFileName << target_dir << "/" << baseFileName << "_" << n << "_foldcv_set-" << i << "_testing";
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
		trainingFileName << target_dir << "/" << baseFileName << "_" << n << "_foldcv_set-" << i << "_training";
		write_netcdf_file (trainingFileName.str(), partitions_training[i]);
		
	}

	
}

bool generate_network_files_nfoldcv_set (const string defaultnetFileName, const string baseDataFileName, int n, string target_dir );

class OfflineRNN {
	int totalEpochs;
	int randomSeed;
	double initWeightRange;
	double trainDataFraction;
	double testDataFraction;
	double valDataFraction;
	//string trainDataFile;
	vector<string> trainDataFiles;
	string testDataFile;
	string valDataFile;
	bool initErrorTest;
	bool gradTest;
	double gradTestPerturbation;
	int gradTestSeq;
	NetcdfDataset* trainData;
	NetcdfDataset* testData;
	NetcdfDataset* valData;
	int epochsPerErrCheck;
	int epochsSinceErrCheck;
	bool batchLearn;
	int maxTestsNoBest; 
	bool overwriteSaves;
	bool shuffleTrainData;
	double staticNoise;	
	//double minStopError;
	int epoch; 
	int saveAfterNSeqs;
	int seqsPerWeightUpdate;
	bool rProp;
	string testOutputsFile;
	bool dataCheck;
	double bestStopError;
	int testsSinceBest;

	Net* net;
	GradientFollower* gradientFollower;
	map<const string, pair<int,double> > netErrorMap;
	vector<string> criteria;
	map <const string, pair<double, int> > bestValErrors;
	vector<string> bestValStrings;
	map <const string, pair<double, int> > bestTestErrors;
	vector<string> bestTestStrings;
	map <const string, pair<double, int> > actualTestErrors;

	const DOMElement* rootNode;
	const DOMElement* consoleDataElement;
	const DOMElement* netElement;
	const DOMElement* exportDataElement;
	
 public:

	OfflineRNN () {
		//data loaded in from xml file (default values below)
		SequenceDebugOutput::instance().set (false);
		totalEpochs = -1;
		randomSeed = 0;
		initWeightRange = -1;
		trainDataFraction = 1;
		testDataFraction = 1;
		valDataFraction = 1;
		//trainDataFile = "";
		testDataFile = "";
		valDataFile = "";
		initErrorTest = false;
		gradTest = false;
		gradTestPerturbation = 1e-5;
		gradTestSeq = -1;
		trainData = 0;
		testData = 0;
		valData = 0;
		epochsPerErrCheck = 5;
		epochsSinceErrCheck = 0;
		batchLearn = false;
		maxTestsNoBest = 4; 
		overwriteSaves = true;
		shuffleTrainData = true;
		staticNoise = 0;	
		//minStopError = 0;
		epoch = 0; 
		saveAfterNSeqs = 0;
		seqsPerWeightUpdate = 1;
		rProp = false;
		testOutputsFile = "";
		dataCheck = false;
	}

	bool parse_netfile (XercesDOMParser& parser, const string& filename, bool display = false);
	
	bool load_net (int displaySeq = -1, const string& displayDataSet = "");
	
	void print_net_data (ostream& out = cout);
	
	void saveConsoleData(ostream& out = cout, bool xml = true);

	void save_net (ostream& out = cout);

	void setTestDataFile (string fileName);

	void setTrainDataFile (string fileName);
};

}; /* smlearning namespace */

#endif /* TOOLS_DATAHANDLING_H_*/
