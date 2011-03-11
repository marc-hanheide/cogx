#include <metalearning/data_structs.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	if (argc < 2) {
		cerr << argv[0] << " sequence_file (without extension)" << endl;
		return 1;
	}
	
	LearningData::DataSet savedData;
	LearningData::FeaturesLimits limits;
	string seqFile = string (argv[1]);

	if (!LearningData::read_dataset (seqFile, savedData, limits)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	LearningData::print_dataset_limits (limits);
	LearningData::print_dataset (savedData);
	

	return 0;

}
