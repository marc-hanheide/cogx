#include <metalearning/data_structs.h>
#include <metalearning/RNN.h>

using namespace smlearning;

int main(int argc, char * argv[]) {
	if (argc < 5) {
		cerr << argv[0] << " sequence_file (without extension) target_dir basis/markov nr_cv_sets" << endl;
		return 1;
	}

	LearningData::DataSet savedData;
	LearningData::FeaturesLimits limits;
	string seqFile = string (argv[1]);
	string target_dir = string (argv[2]);
	string featureselection = string (argv[3]);
	int n = atoi (argv[4]);
	if (n < 2)
		return 1;
	
	if (!LearningData::read_dataset (seqFile, savedData, limits)) {
		cerr << "error reading data" << endl;
		return 1;
	}
	// if (!read_dataset (seqFile, savedDataold)) {
	// 	cerr << "error reading data" << endl;
	// 	return 1;
	// }

	string seqBaseFileName = get_seqBaseFileName (seqFile);

	//print_dataset<double> (savedData);
	//writing to nc file
	// if (featureselection == "basis") {
	// 	if (LearningData::write_nc_file_NNbasis/*write_nc_file_basis*/ (target_dir + "/" + seqBaseFileName, savedData/*old*/, normalize<Real>, limits))
	// 		cout << "nc file written" << endl;
	// 	else
	// 		cerr << "nc file NOT written" << endl;
	// }
	// else if (featureselection == "padding") {
	// 	if (write_cdl_file_padding (target_dir + "/" + seqBaseFileName, savedData))
	// 		cout << "nc file written" << endl;
	// 	else
	// 		cerr << "nc file NOT written" << endl;
	// }
 	// else if (featureselection == "markov") {
	// 	if (LearningData::write_nc_file_NNmarkov (target_dir + "/" + seqBaseFileName, savedData))
	// 		cout << "nc file written" << endl;
	// 	else
	// 		cerr << "nc file NOT written" << endl;
	// }
	//generate n fold cross validation sets
	if (featureselection == "basis")
		LearningData::write_n_fold_cross_valid_sets (seqFile, n, LearningData::write_nc_file_NNbasis<Real(*)(Real const&, Real const&, Real const&) >, normalize<Real>, limits, target_dir );
	// else if (featureselection == "padding")
	// 	write_n_fold_cross_valid_sets (seqFile, n, write_cdl_file_padding, target_dir );
	else if (featureselection == "markov")
		LearningData::write_n_fold_cross_valid_sets (seqFile, n, LearningData::write_nc_file_NNmarkov<Real(*)(Real const&, Real const&, Real const&) >, normalize<Real>, limits, target_dir );

	generate_network_files_nfoldcv_set ("/usr/local/bin/SMLearning/defaultnet.config", seqBaseFileName, n, target_dir );

	return 0;
}
