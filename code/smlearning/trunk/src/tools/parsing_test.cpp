#include <metalearning/RNN.h>

using namespace smlearning;

int main(int argc, char * argv[]) {

	RNN myRNN;

	rnnlib::ConfigFile conf("/usr/local/bin/SMLearning/defaultnet.config");
	myRNN.set_config_file (conf);
	myRNN.set_testdatafile ("/usr/local/bin/SMLearning/valid.nc");
	myRNN.set_traindatafile ("/usr/local/bin/SMLearning/train.nc");
	myRNN.init ();
	// myRNN.print_net_data ();
	myRNN.write_config_file (cout);
	return 0;
}
