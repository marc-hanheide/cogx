#include <tools/data_handling.h>

using namespace smlearning;

int main (int argc, char* argv[]) {

	if (argc < 2) {
		cerr << argv[0] << " sequence_file (without extension)" << endl;
		return 1;
	}
	
	DataSet savedData;
	string seqFile = string (argv[1]);
// 	string target_dir = string (argv[2]);

	if (!read_dataset (seqFile, savedData)) {
		cerr << "error reading data" << endl;
		return 1;
	}

// 	string seqBaseFileName = get_seqBaseFileName (seqFile);

// 	CanonicalData::DataSet newData = canonical_input_output_enumerator (savedData);
	CanonicalData::DataSet newData = canonical_input_output_enumerator_with_time (savedData);

// 	write_canonical_dataset (target_dir + "/" + seqBaseFileName + "-disc", newData);
// 	write_canonical_dataset_cryssmex_fmt (target_dir + "/" + seqBaseFileName, newData);
	write_canonical_dataset_cryssmex_fmt (seqFile, newData);

	return 0;
}
