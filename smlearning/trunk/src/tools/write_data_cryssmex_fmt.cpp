#include <metalearning/data_structs.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	string seqFile;
	string target_dir = "./";
	// int modulo = 0;
	unsigned int featureSelectionMethod;
	if (argc >= 4) {
		target_dir = string (argv[3]);
	}
	if (argc >= 3) {
		string fSMethod = string (argv[2]);
		if (fSMethod == "obpose")
			featureSelectionMethod = _obpose;
		else if (fSMethod == "efobpose")
			featureSelectionMethod = _efobpose;
		else {
			cerr << "Please type obpose or efobpose as second argument" << endl;
			return 1;
		}
		seqFile = string (argv[1]);
	}
	// if (argc >= 3) {
	// 	modulo = atoi(argv[2]);
	// 	cout << modulo << endl;
	// }
	else {
		// cerr << argv[0] << " sequence_file (without extension) [modulo] [target_dir (default:current file dir.)]" << endl;
		cerr << argv[0] << " sequence_file (without extension) obpose/efobpose [target_dir (default:current file dir.)]" << endl;
		return 1;
	}

	// if (modulo <= 0) {
	// 	cout << "modulo assumed to be 1" << endl;
	// 	modulo = 1;
	// }


	LearningData::DataSet savedData;
	LearningData::CoordinateLimits limits;

	if (!LearningData::read_dataset (seqFile, savedData, limits)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);

	//if (argc == 4)
		LearningData::write_cryssmexdataset_regression (target_dir + "/" + seqBaseFileName, savedData, normalize<double>, limits, featureSelectionMethod/*, modulo*/);
	//else if (/*argc == 3 || */ argc == 3)
		//LearningData::write_cryssmexdataset_regression (seqFile, savedData, normalize<double>, limits, featureSelectionMethod/*, modulo*/);

	return 0;
}
