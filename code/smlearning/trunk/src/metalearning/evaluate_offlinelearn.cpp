#include <metalearning/SMRegion.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	string seqFile;
	string netFile;
	if (argc == 3) {
		seqFile = string (argv[1]);
		netFile = string (argv[2]);
	}
	else {
		cerr << argv[0] << " sequence_file (without extension) net_file" << endl;
		return 1;
	}

	DataSet data;

	if (!read_dataset (seqFile, data)) {
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
		rnnlib::DataSequence* testSeq = load_trainSeq (data[i], SMRegion::motorVectorSize + SMRegion::featureVectorSize, SMRegion::pfVectorSize);
		error += myRNN.net->calculate_errors (*testSeq);
	
	}
	error /= (double)data.size();

	cout << "Avg. sum of squares error: " << error << endl;

 	return 0;


}
