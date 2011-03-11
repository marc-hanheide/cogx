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
		else if (fSMethod == "obpose_label")
			featureSelectionMethod = _obpose_label;
		else if (fSMethod == "efobpose_label")
			featureSelectionMethod = _efobpose_label;
		else {
			cerr << "Please type obpose, obpose_label, efobpose or efobpose_label as second argument" << endl;
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
		cerr << argv[0] << " sequence_file (without extension) obpose[_label]/efobpose[_label] [target_dir (default:current file dir.)]" << endl;
		return 1;
	}

	// if (modulo <= 0) {
	// 	cout << "modulo assumed to be 1" << endl;
	// 	modulo = 1;
	// }


	LearningData::DataSet savedData;
	LearningData::FeaturesLimits limits;

	if (!LearningData::read_dataset (seqFile, savedData, limits)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);

	//if (argc == 4)
		LearningData::write_cryssmexdataset (target_dir + "/" + seqBaseFileName, savedData, normalize<double>, limits, featureSelectionMethod/*, modulo*/);
	//else if (/*argc == 3 || */ argc == 3)
		//LearningData::write_cryssmexdataset_regression (seqFile, savedData, normalize<double>, limits, featureSelectionMethod/*, modulo*/);

	return 0;
}
