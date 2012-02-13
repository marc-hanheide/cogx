#include <tools/data_handling.h>

using namespace smlearning;

int main (int argc, char *argv[]) {

	string dir;
	if (argc == 3)
		dir = string (argv[2]);
	else if (argc == 2)
		dir = "./";
	else {
		cerr << argv[0] << " write_sequence_file_name (without extension) [target_dir (default:current dir.)]" << endl;
		exit (0);
	}
	
	if (concatenate_datasets ( dir, argv[1] ))
		cout << "Successful concatenation." << endl;
	else {
		cerr << "Error in concatenation." << endl;
		return 1;
	}
			
	return 0;
}