#include <metalearning/data_structs.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;


using namespace smlearning;

int main (int argc, char* argv[]) {

	string seqFile;
	string target_dir;
	// int modulo = 0;
	feature_selection featureSelectionMethod;

	po::options_description desc("Allowed parameters:");
	desc.add_options()
		("help,h", "produce help message")
		("seqfile,s", po::value (&seqFile), "Sequence file (without extension).\n(Required parameter)")
		("targetdir,t", po::value (&target_dir)->default_value ("./"), "Target directory")
		("featuresel,f", po::value (&featureSelectionMethod)->default_value (_mcobpose_obpose_direction), feature_selection_options ().c_str())
		("normalize,n", "Normalize data");

	// Declare an options description instance which will include
	// all the options
	po::options_description all("Basic usage:");
	all.add(desc);
  
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
		  options(all).run(), vm);
	po::notify(vm);

	if (vm.count("help") || !vm.count("seqfile")) {
		cout << desc << "\n";
		return 0;
	}

	// if (modulo <= 0) {
	// 	cout << "modulo assumed to be 1" << endl;
	// 	modulo = 1;
	// }

	LearningData::DataSet savedData;
	LearningData::FeaturesLimits limits;

	if (!LearningData::read_dataset (seqFile, savedData, limits)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	string seqBaseFileName = get_seqBaseFileName (seqFile);


	if (vm.count("normalize"))
		LearningData::write_cryssmexdataset (target_dir + "/" + seqBaseFileName, savedData, normalize<double>, limits, featureSelectionMethod/*, modulo*/);
	else
		LearningData::write_cryssmexdataset (target_dir + "/" + seqBaseFileName, savedData, donotnormalize<double>, limits, featureSelectionMethod/*, modulo*/);
		



	return 0;
}
