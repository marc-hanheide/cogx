#include <metalearning/data_structs.h>
#include <metalearning/SMRegion.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	string seqFile;
	string netFile;
	unsigned int featureSelectionMethod;
	if (argc == 4) {
		seqFile = string (argv[1]);
		netFile = string (argv[2]);
		string fSMethod = string (argv[3]);
		if (fSMethod == "basis")
			featureSelectionMethod = _basis;
		else if (fSMethod == "markov")
			featureSelectionMethod = _markov;			
	}
	else {
		cerr << argv[0] << " sequence_file (without extension) net_file basis/markov" << endl;
		return 1;
	}

	LearningData::DataSet data;
	LearningData::CoordinateLimits limits;
	

	if (!LearningData::read_dataset (seqFile, data, limits )) {
		cerr << "error reading data" << endl;
		return 1;
	}
	
	RNN myRNN;

	cout << "netFile: " << netFile << endl;

	rnnlib::ConfigFile conf(netFile);
	myRNN.set_config_file (conf);
	myRNN.init ();

	double error = 0.0;

	for (int i = 0; i < data.size(); i++) {
		rnnlib::DataSequence* testSeq = LearningData::load_NNtrainSeq (data[i], featureSelectionMethod, normalize<double>, limits );
		error += myRNN.net->calculate_errors (*testSeq);
	
	}
	error /= (double)data.size();

	cout << "Avg. sum of squares error: " << error << endl;

 	return 0;


}
