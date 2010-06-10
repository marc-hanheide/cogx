#include <tools/data_handling.h>
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

	DataSetStruct data;

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

	for (int i = 0; i < data.first.size(); i++) {
		//DataSetParams params = make_tuple ((int)(SMRegion::motorVectorSize + SMRegion::featureVectorSize), (int)SMRegion::pfVectorSize, (int)SMRegion::motorVectorSize, (int)SMRegion::efVectorSize, false);
		rnnlib::DataSequence* testSeq = load_trainSeq (data.first[i], data.second);
		error += myRNN.net->calculate_errors (*testSeq);
	
	}
	error /= (double)data.first.size();

	cout << "Avg. sum of squares error: " << error << endl;

 	return 0;


}
