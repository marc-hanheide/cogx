#include <tools/data_handling.h>

int main(int argc, char * argv[]) {
	if (argc < 3) {
		cerr << argv[0] << " [nr. of sequences] [sequences size]" << endl;
		exit (0);
	}
	DataSet data;

	//Generate artificial random sequences
	int numSeq = atoi(argv[1]);
	int seqSize = atoi(argv[2]);
	generate_rand_sequences (data, numSeq, seqSize);
	cout << "printing generated data: " << endl;
	print_dataset<double> (data);
	write_dataset ("training", data);
	cout << "printing stored data: " << endl;
	DataSet savedData;
	read_dataset ("training", savedData);
	print_dataset<double> (savedData);
	//writing to cdl file
	if (write_cdl_file_padding ("training", savedData))
		cout << "cdl file written" << endl;
	else
		cout << "cdl file NOT written" << endl;
	//generate n fold cross validation sets
	const int n = 5;
	write_n_fold_cross_valid_sets ("training", n, write_cdl_file_padding);

	
}
