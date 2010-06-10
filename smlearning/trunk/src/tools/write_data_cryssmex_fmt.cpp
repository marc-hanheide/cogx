#include <tools/data_handling.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	// string seqFile;
	// string target_dir;
	// int modulo = 0;
	// if (argc == 3) {
	// 	target_dir = string (argv[2]);
	// }
	// if (argc >= 2) {
	// 	seqFile = string (argv[1]);
	// }
	// else {
	// 	cerr << argv[0] << " [sequence_file (without extension)] [modulo] [target_dir (default:current file dir.)]" << endl;
	// 	return 1;
	// }

	string seqFile;
	string target_dir;
	int modulo = 0;
	if (argc >= 4) {
		target_dir = string (argv[3]);
	}
	if (argc >= 3) {
		modulo = atoi(argv[2]);
		cout << modulo << endl;
	}
	if (argc >= 2)
		seqFile = string (argv[1]);
	else {
		cerr << argv[0] << " [sequence_file (without extension)]  [modulo] [target_dir (default:current file dir.)]" << endl;
		return 1;
	}

	if (modulo <= 0) {
		cout << "modulo assumed to be 1" << endl;
		modulo = 1;
	}


	DataSetStruct savedData;

	if (!read_dataset (seqFile, savedData)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);

	if (argc == 4)
		write_dataset_cryssmex_fmt_with_label (target_dir + "/" + seqBaseFileName, savedData.first, modulo);
	else if (argc == 3 || argc == 2)
		write_dataset_cryssmex_fmt_with_label (seqFile, savedData.first, modulo);

	return 0;
}
