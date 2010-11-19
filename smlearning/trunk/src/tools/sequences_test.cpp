#include <tools/data_handling.h>

using namespace smlearning;

#define FEATUREVECTOR_SIZE1 8
#define FEATUREVECTOR_SIZE2 6

///
///generation of random sequences (for testing purposes)
///
void generate_rand_sequences (DataSet& data, long numSeq, long seqSize) {
	// initialize random seed:
	srand ((unsigned)time(NULL) );
	
	// generate random number:
	double randNr;

	for (int s=0; s<numSeq; s++) {
		smlearning::Sequence currentSequence;
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


int main(int argc, char * argv[]) {
	if (argc < 2) {
		cerr << argv[0] << " seq_file (without extension)" << endl;
		return 1;
	}
	//DataSetStruct savedData;
	LearningData::DataSet savedData;
	LearningData::CoordinateLimits limits;

	//Generate artificial random sequences
// 	int numSeq = atoi(argv[1]);
// 	int seqSize = atoi(argv[2]);
// 	generate_rand_sequences (data, numSeq, seqSize);

	string seqFile = string (argv[1]);
	if (!LearningData::read_dataset (seqFile, savedData, limits)) {
		cout << argv[1] << ".seq2" << " does not exist!" << endl;
		return 1;
	}
// 	cout << "printing generated data: " << endl;
// 	print_dataset<double> (data);
// 	write_dataset ("training", data);
// 	cout << "printing stored data: " << endl;
// 	DataSet savedData;
// 	read_dataset ("training", savedData);
// 	print_dataset<double> (savedData);
	//writing to cdl file
// 	if (write_cdl_file_padding ("training", savedData))
// 	if (write_cdl_file_basis (seqFile, savedData))
// 		cout << "cdl file written" << endl;
// 	else
// 		cout << "cdl file NOT written" << endl;
	if (LearningData::write_nc_file_NNbasis (seqFile, savedData, normalize<Real>, limits))
		cout << "nc file written" << endl;
	else
		cout << "nc file NOT written" << endl;
	//generate n fold cross validation sets
// 	const int n = 10;
// 	write_n_fold_cross_valid_sets ("training", n, write_cdl_file_padding);

	return 0;
	
}
