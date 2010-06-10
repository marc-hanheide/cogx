#include <tools/data_handling.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	if (argc < 2) {
		cerr << argv[0] << " sequence_file (without extension)" << endl;
		return 1;
	}
	
	DataSetStruct savedData;
	string seqFile = string (argv[1]);

	if (!read_dataset (seqFile, savedData)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	print_dataset<double> (savedData.first);
	print_dataset_params (savedData.second);

	return 0;

}
