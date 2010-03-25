#include <tools/data_handling.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

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
	
	DataSet savedData;

	if (!read_dataset (seqFile, savedData)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);

// 	CanonicalData::DataSet newData = canonical_input_output_enumerator (savedData);
	CanonicalData::DataSet newData = canonical_input_output_enumerator_with_time (savedData, modulo);

	if (argc == 4)
		write_canonical_dataset_cryssmex_fmt (target_dir + "/" + seqBaseFileName, newData);
	else if (argc == 3 || argc == 2)
		write_canonical_dataset_cryssmex_fmt (seqFile, newData);

	return 0;
}
