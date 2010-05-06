#include <tools/data_handling.h>
#include <metalearning/RNN.h>

using namespace smlearning;

int main(int argc, char * argv[]) {
	if (argc < 5) {
		cerr << argv[0] << " sequence_file (without extension) target_dir basis/padding/Markov nr_cv_sets" << endl;
		return 1;
	}

	DataSet savedData;
	string seqFile = string (argv[1]);
	string target_dir = string (argv[2]);
	string encoding = string (argv[3]);
	int n = atoi (argv[4]);
	if (n < 2)
		return 1;
	
	if (!read_dataset (seqFile, savedData)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);

	//print_dataset<double> (savedData);
	//writing to nc file
	if (encoding == "basis") {
		if (write_nc_file_basis (target_dir + "/" + seqBaseFileName, savedData))
			cout << "nc file written" << endl;
		else
			cout << "nc file NOT written" << endl;
	}
	else if (encoding == "padding") {
		if (write_cdl_file_padding (target_dir + "/" + seqBaseFileName, savedData))
			cout << "nc file written" << endl;
		else
			cout << "nc file NOT written" << endl;
	}
	else if (encoding == "Markov") {
		if (write_nc_file_Markov (target_dir + "/" + seqBaseFileName, savedData))
			cout << "nc file written" << endl;
		else
			cout << "nc file NOT written" << endl;
	}
	//generate n fold cross validation sets
	if (encoding == "basis")
		write_n_fold_cross_valid_sets (seqFile, n, write_nc_file_basis, target_dir );
	else if (encoding == "padding")
		write_n_fold_cross_valid_sets (seqFile, n, write_cdl_file_padding, target_dir );
	else if (encoding == "Markov")
		write_n_fold_cross_valid_sets (seqFile, n, write_nc_file_Markov, target_dir );

	generate_network_files_nfoldcv_set ("/usr/local/bin/SMLearning/defaultnet.config", seqBaseFileName, n, target_dir );

	return 0;
}
