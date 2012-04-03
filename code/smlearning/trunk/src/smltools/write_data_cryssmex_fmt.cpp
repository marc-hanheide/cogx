#include <metalearning/data_structs.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;


using namespace smlearning;

int main (int argc, char* argv[]) {

	string seqFile;
	string target_dir;
	// int modulo = 0;
	string fSMethod;
	unsigned int featureSelectionMethod;

	po::options_description desc("Allowed parameters:");
	desc.add_options()
		("help,h", "produce help message")
		("seqfile,s", po::value (&seqFile), "Sequence file (without extension).\n(Required parameter)")
		("targetdir,t", po::value (&target_dir)->default_value ("./"), "Target directory")
		("featuresel,f", po::value (&fSMethod)->default_value ("efobpose"), "Feature selection method\n(obpose, obpose_label,\nobpose_direction, obpose_slide_flip_tilt\nefobpose, efobpose_label,\nefobpose_direction, efobpose_slide_flip_tilt)")
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

	if (fSMethod == "obpose")
		featureSelectionMethod = _obpose;
	else if (fSMethod == "efobpose")
		featureSelectionMethod = _efobpose;
	else if (fSMethod == "obpose_label")
		featureSelectionMethod = _obpose_label;
	else if (fSMethod == "efobpose_label")
		featureSelectionMethod = _efobpose_label;
	else if (fSMethod == "obpose_direction")
		featureSelectionMethod = _obpose_direction;
	else if (fSMethod == "efobpose_direction")
		featureSelectionMethod = _efobpose_direction;
	else if (fSMethod == "obpose_rough_direction")
		featureSelectionMethod = _obpose_rough_direction;
	else if (fSMethod == "efobpose_rough_direction")
		featureSelectionMethod = _efobpose_rough_direction;
	else if (fSMethod == "obpose_slide_flip_tilt")
		featureSelectionMethod = _obpose_slide_flip_tilt;
	else if (fSMethod == "efobpose_slide_flip_tilt")
		featureSelectionMethod = _efobpose_slide_flip_tilt;
	else {
		cout << desc << "\n";
		return 1;
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
