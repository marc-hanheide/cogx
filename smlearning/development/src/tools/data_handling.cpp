#include <tools/data_handling.h>

#define FEATUREVECTOR_SIZE1 8
#define FEATUREVECTOR_SIZE2 6

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

void generate_rand_sequences (DataSet& data, long numSeq, long seqSize) {
	// initialize random seed:
	srand ( time(NULL) );
	
	// generate random number:
	double randNr;

	for (int s=0; s<numSeq; s++) {
		Sequence currentSequence;
		for (int v=0; v<seqSize; v++) {

			FeatureVector currentVector;
			int vectorSize;
			if (v==0)
				vectorSize = FEATUREVECTOR_SIZE1;
			else
				vectorSize = FEATUREVECTOR_SIZE2;
		
			for (int n=0; n< vectorSize; n++) {
				randNr = (rand() % 10 + 1) / 10.0;
				currentVector.push_back (randNr);			
			}

			
			currentSequence.push_back (currentVector);	
		}
		data.push_back (currentSequence);
	}

}

bool write_dataset (string fileName, const DataSet& data) {
	fileName += ".seq";
	ofstream writeFile(fileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;

	long numSeqs = data.size();
	writeFile.write ((const char*)&numSeqs, sizeof(numSeqs));
//  	cout << numSeqs << endl;
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		long seqSize = (*s).size();
		writeFile.write ((const char*)&seqSize, sizeof (seqSize));
//  		cout << "\t" << seqSize << endl;
		Sequence::const_iterator v;

		for (v=(*s).begin(); v!= (*s).end(); v++) {
			long featvectorSize = (*v).size();
			writeFile.write ((const char*)&featvectorSize, sizeof (featvectorSize));
//  			cout << "\t\t" << featvectorSize << endl;
			FeatureVector::const_iterator n;
			for (n=(*v).begin(); n!= (*v).end(); n++) {
				writeFile.write ((const char* )&(*n), sizeof (*n));
			}
		}
	}
	
	writeFile.close();
	return true;
}

bool read_dataset (string fileName, DataSet& data) {
	fileName += ".seq";
	ifstream readFile(fileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;

	long numSeq;
	readFile.read ((char* )&numSeq, sizeof(numSeq));
//  	cout << numSeq << endl;
	for (int s=0; s<numSeq; s++) {
		Sequence currentSequence;
		long seqSize;
		readFile.read((char *)&seqSize, sizeof(seqSize));
//  		cout << "\t" << seqSize << endl;
		for (int v=0; v<seqSize; v++) {
			FeatureVector currentVector;
			long featvectorSize;
			readFile.read ((char *)&featvectorSize, sizeof(featvectorSize));
// 			cout << "\t\t" << featvectorSize << endl;
			for (int n=0; n<featvectorSize; n++) {
				double value;
				readFile.read ((char* )&value, sizeof(value));
				currentVector.push_back (value);
			}
			currentSequence.push_back (currentVector);
		}
		data.push_back(currentSequence);
	}	
	
	readFile.close();
	return true;



	
}

//write a cdl file format with zero padding
bool write_cdl_file_padding (string fileName, const DataSet& data) {
	fileName += ".cdl";
	ofstream writeFile(fileName.c_str(), ios::out);
	if (!writeFile)
		return false;

	long maxfeatvectorSize = -1;

	//find max. feature vector size which will be inputPattSize and targetPattSize
	Sequence::const_iterator v = (*(data.begin())).begin();
	for (int i=0; i<2; i++,v++) {
		long featvectorSize = (*v).size();
		if (featvectorSize > maxfeatvectorSize)
			maxfeatvectorSize = featvectorSize;
	}
		
	stringstream seqLengthsStr;
	stringstream inputsStr;
	stringstream targetPatternsStr;
	long numTimesteps = 0;
	seqLengthsStr << "\tseqLengths = ";
	inputsStr << "\tinputs =" << endl << "\t\t";
	targetPatternsStr << "\ttargetPatterns = " << endl << "\t\t";
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		long seqSize = (*s).size() - 1;
		seqLengthsStr << seqSize;
		numTimesteps += seqSize;
		if (s+1 == data.end())
			seqLengthsStr << ";";
		else
			seqLengthsStr << ", ";

		Sequence::const_iterator v;
		for (v=(*s).begin(); v!= (*s).end(); v++) {
			long featvectorSize = (*v).size();
			long paddingSize = maxfeatvectorSize - featvectorSize;
			FeatureVector::const_iterator n;
			//put inputs and targetPatterns data
			if (v+1 != (*s).end()) {
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					if (n != (*v).begin())
						inputsStr << ", ";
					inputsStr << *n;
				}
				//zero padding
				for (int i=0; i<paddingSize; i++)
					inputsStr << ", 0";
				if (v+2 == (*s).end() && s+1 == data.end())
					inputsStr << ";";
				else
					inputsStr << "," << endl << "\t\t";
			}
			if (v != (*s).begin()) {
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					if (n != (*v).begin())
						targetPatternsStr << ", ";
					targetPatternsStr << *n;
				}
				//zero padding
				for (int i=0; i<paddingSize; i++)
					targetPatternsStr << ", 0";
				if (v+1 == (*s).end() && s+1 == data.end())
					targetPatternsStr << ";";
				else
					targetPatternsStr << "," << endl << "\t\t";
			}

			
		}
	}

	writeFile << "netcdf " << fileName << " {" << endl;
	int numSeqs = data.size();
	writeFile << "dimensions:" << endl;
	writeFile << "\tnumSeqs = " << numSeqs << ",\n";
	writeFile << "\tnumTimesteps = " << numTimesteps << "," << endl;
	writeFile << "\tinputPattSize = " << maxfeatvectorSize << "," << endl;
	writeFile << "\ttargetPattSize = " << maxfeatvectorSize << ";" << endl;
	writeFile << "variables:" << endl << "\tfloat inputs(numTimesteps, inputPattSize);" \
		  << endl << "\tint seqLengths(numSeqs);" << endl \
		  << "\tfloat targetPatterns(numTimesteps, targetPattSize);" << endl;
	writeFile << "data:" << endl;
	writeFile << inputsStr.str() << endl;
	writeFile << seqLengthsStr.str() << endl;
	writeFile << targetPatternsStr.str() << endl;
	writeFile << "}" << endl;

	writeFile.close ();
	return true;
}

template<class Function>
bool write_n_fold_cross_valid_sets (string baseFileName, int n, Function write_cdl_file) {

	DataSet data;
	if (!read_dataset (baseFileName, data))
		return false;
	
	vector<DataSet> partitions_testing;
	vector<DataSet> partitions_training;
	
	// initialize random seed:
	srand ( time(NULL) );
	
	// generate random number:
	double randNr;

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
		print_dataset<double>(partitions_testing[i]);
		stringstream testingFileName;
		testingFileName << baseFileName << "_" << n << "_foldcv_set-" << i << "_testing";
		write_cdl_file (testingFileName.str(), partitions_testing[i]);
		for (int j=0; j<n; j++)
			if (i != j) {
				DataSet::const_iterator s;
				for (s=partitions_testing[j].begin(); s!=partitions_testing[j].end(); s++)
					partitions_training[i].push_back (*s);
			}
		cout << "size of training partition " << i << ": " << partitions_training[i].size() << endl;
		print_dataset<double>(partitions_training[i]);
		stringstream trainingFileName;
		trainingFileName << baseFileName << "_" << n << "_foldcv_set-" << i << "_training";
		write_cdl_file (trainingFileName.str(), partitions_training[i]);
		
	}


}

int main(int argc, char * argv[]) {
	if (argc < 3) {
		cerr << argv[0] << " [nr. of sequences] [sequences size]" << endl;
		exit (0);
	}
	DataSet data;

	//Generate artificial random sequences
	int numSeq = atoi(argv[1]);
	int seqSize = atoi(argv[2]);
	generate_rand_sequences (data, numSeq, seqSize);
	cout << "printing generated data: " << endl;
	print_dataset<double> (data);
	write_dataset ("training", data);
	cout << "printing stored data: " << endl;
	DataSet savedData;
	read_dataset ("training", savedData);
	print_dataset<double> (savedData);
	//writing to cdl file
	if (write_cdl_file_padding ("training", savedData))
		cout << "cdl file written" << endl;
	else
		cout << "cdl file NOT written" << endl;
	//generate n fold cross validation sets
	const int n = 5;
	write_n_fold_cross_valid_sets ("training", n, write_cdl_file_padding);

	
}
