#include <metalearning/OfflineRNN.h>

using namespace smlearning;

int main(int argc, char * argv[]) {

	OfflineRNN myRNN;

	rnnlib::ConfigFile conf("/usr/local/bin/SMLearning/defaultnet.config");
	myRNN.set_config_file (conf);

	myRNN.load_net ();
	myRNN.print_net_data ();
	myRNN.save_config_file (cout);
	return 0;
}
