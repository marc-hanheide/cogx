#include <tools/data_handling.h>

using namespace smlearning;

int main(int argc, char * argv[]) {
	if (argc < 2) {
		cerr << argv[0] << " seq_file (without extension)" << endl;
		exit (0);
	}
	DataSet savedData;

	//Generate artificial random sequences
// 	int numSeq = atoi(argv[1]);
// 	int seqSize = atoi(argv[2]);
// 	generate_rand_sequences (data, numSeq, seqSize);

	string seqFile = string (argv[1]);
	if (!read_dataset (seqFile, savedData)) {
		cout << argv[1] << ".seq" << " does not exist!" << endl;
		return 1;
	}
// 	cout << "printing generated data: " << endl;
// 	print_dataset<double> (data);
// 	write_dataset ("training", data);
// 	cout << "printing stored data: " << endl;
// 	DataSet savedData;
// 	read_dataset ("training", savedData);
// 	print_dataset<double> (savedData);
	//writing to cdl file
// 	if (write_cdl_file_padding ("training", savedData))
// 	if (write_cdl_file_basis (seqFile, savedData))
// 		cout << "cdl file written" << endl;
// 	else
// 		cout << "cdl file NOT written" << endl;
	if (write_nc_file_basis (seqFile, savedData))
		cout << "nc file written" << endl;
	else
		cout << "nc file NOT written" << endl;
	//generate n fold cross validation sets
// 	const int n = 10;
// 	write_n_fold_cross_valid_sets ("training", n, write_cdl_file_padding);

	return 0;
	
}
