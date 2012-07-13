/** @file onlinelearningdataset_experiment.cpp
 *
 *
 *
 * @author      Sergio Roa - DFKI
 * @version 2.0 beta
 *
 */

#include <scenario/OnlineLearningDataset.h>
#include <boost/program_options.hpp>
#include <Golem/Phys/Application.h>

using namespace smlearning;
namespace po = boost::program_options;


class OnlineLearningDatasetApplication : public golem::Application
{
public:
	virtual int main (int argc, char *argv[])
	{
		try {
			prgOptDesc.add_options()
				("help,h", "produce help message")
				("seqFile,d", po::value<string>(), "name of file containing data sequences\n(do not type .seq extension)")
				("maxepochsmdl,e", po::value<unsigned int>()->default_value (100), "Set maximum nr of epochs after mdl reduction is expected (for GNG based quantizing). This value should be proportionally changed depending on maxepochserror parameter.")
				("maxepochserror", po::value<unsigned int>()->default_value (1), "Set maximum nr of epochs after error reduction is expected (for GNG based quantizing)")
				("modelefficiency,c", po::value<double>()->default_value (1), "model efficiency constant (for GNG based quantizing) for input space")
				("output_modelefficiency", po::value<double>()->default_value(10), "model efficiency constant (for GNG based quantizing) for output space")
				("learningrate,l", po::value<double>()->default_value (0.8), "default learning rate for winner nodes (for GNG based quantizing)")
				("accuracy,y", po::value<double>()->default_value(0.0001), "data accuracy constant for input space GNG quantization")
				("output_accuracy", po::value<double>()->default_value (0.0001), "data accuracy constant for output space GNG quantization")
				("featuresel,f", po::value<feature_selection>()->default_value (_mcobpose_obpose_direction), feature_selection_options ().c_str())
				("splitting,s", po::value<unsigned int>()->default_value (30), "Splitting criterion (nr of sequences needed for splitting")
				("mdl,m", "save MDL history");


			po::store(po::parse_command_line(argc, argv, prgOptDesc), vm);
			po::notify(vm);

			if (vm.count("help")) {
			
				cout << prgOptDesc << endl;
				return 0;
			}
		}catch(std::exception& e) {
			cerr << "error: " << e.what() << "\n";
			return 1;
		}catch(...) {
			cerr << "Exception of unknown type!\n";
			return 1;

		}

		char* arr [] = {argv[0]};
		Application::main (1, arr);
		return 0;
		
	}

	virtual void run(int argc, char *argv[])
	{

		OnlineLearningDataset *experiment = new OnlineLearningDataset (*scene());

		if (experiment == NULL) {
			context()->getMessageStream()->write(Message::LEVEL_CRIT, "unable to cast to OnlineLearningDataset");
			return;
		}
		experiment->init (vm);
		experiment->run(argc, argv);
		delete experiment;

	}
	boost::program_options::variables_map vm;
	boost::program_options::options_description prgOptDesc;

};


int main(int argc, char *argv[]) {

	OnlineLearningDatasetApplication().main (argc, argv);
	return 0;
}
