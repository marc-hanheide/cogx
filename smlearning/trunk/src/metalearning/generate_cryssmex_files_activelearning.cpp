/** @file generate_cryssmex_files_activelearning.cpp
 *
 * Generation of CrySSMEx SSM and CVQ files from input and output quantizers
 * trained by using class \p ActiveLearnScenario
 *
 * @author      Sergio Roa - DFKI
 * @version 2.0 beta
 *
 */

#include <boost/program_options.hpp>
#include <metalearning/GNGSMRegion.h>

namespace po = boost::program_options;

using namespace smlearning;

int main (int argc, char* argv[])
{
	string prefix;
	unsigned int max_iterations;
	ssm::SSM::Type ssm; // = {mealy, moore}
	// bool save_all;
	string seqFile;
	string fSMethod;
	feature_selection featureSelectionMethod;
	
	po::options_description desc("Allowed parameters:");
	desc.add_options()
		("help,h", "produce help message")
		("prefix,p", po::value(&prefix)->default_value("./"), "Prefix for results to be stored.")
		("max_iterations,t", po::value(&max_iterations)->default_value(5), "max number of iterations in the cryssmex main loop before an SSM is found with more states")
		("ssm", po::value(&ssm)->default_value(ssm::SSM::mealy), "[mealy|moore] Extraction of Mealy or Moore machines")
		// ("save_all,a", po::bool_switch(&save_all), "Saves SSMs and CVQs in all iterations")
		("seqFile,d", po::value(&seqFile), "name of file containing data sequences\n(do not type .seq extension)")
		("featuresel,f", po::value(&fSMethod)->default_value ("mcobpose_obpose_direction"), "Feature selection method\n(_obpose|_obpose_label|\n_obpose_direction|_obpose_slide_flip_tilt\n_efobpose|_efobpose_label\n_efobpose_direction|_efobpose_slide_flip_tilt\n_mcobpose_obpose_direction)");

	// Declare an options description instance which will include
	// all the options
	po::options_description all("Basic usage:");
	all.add(desc);
  
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
		  options(all).run(), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 0;
	}

	if (fSMethod == "obpose")
		featureSelectionMethod = _obpose;
	else if (fSMethod == "efobpose")
		featureSelectionMethod = _efobpose;
	else if (fSMethod == "obpose_direction")
		featureSelectionMethod = _obpose_direction;
	else if (fSMethod == "efobpose_direction")
		featureSelectionMethod = _efobpose_direction;
	else if (fSMethod == "mcobpose_obpose_direction")
		featureSelectionMethod = _mcobpose_obpose_direction;
	else if (fSMethod == "obpose_rough_direction")
		featureSelectionMethod = _obpose_rough_direction;
	else if (fSMethod == "efobpose_rough_direction")
		featureSelectionMethod = _efobpose_rough_direction;
	else if (fSMethod == "obpose_slide_flip_tilt")
		featureSelectionMethod = _obpose_slide_flip_tilt;
	else if (fSMethod == "efobpose_slide_flip_tilt")
		featureSelectionMethod = _efobpose_slide_flip_tilt;
	
	LearningData::DataSet data;
	LearningData::FeaturesLimits limits;

	if (!LearningData::read_dataset (seqFile, data, limits)) {
		cerr << "error reading sequence data file" << endl;
		return 1;
	}
	for (int i=0; i<data.size(); i++)
		LearningData::write_chunk_to_featvector (data[i][0].featureVector, data[i][0], normalize<double>, limits, _end_effector_pos | _effector_pos /*| _action_params*/ );

	boost::regex regfile_re ("(.*_final)\\.reg");
	boost::cmatch matches;
	// cout << matches.size() << endl;
	path p(prefix);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		return 1;
	}

	directory_iterator dir_iter (p), dir_end;
	for (;dir_iter != dir_end; ++dir_iter)
	{
		string dirstring (dir_iter->leaf().c_str());
		char *dirchar = (char *)dirstring.c_str();
		if (boost::regex_match ((const char*)dirchar, matches, regfile_re))
		{
			GNGSMRegion currentRegion;
			string regionFileName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << regionFileName << endl;
			if (currentRegion.readData (regionFileName)) {
				cout << "region data correctly read..." << endl << "======" << endl;
			}
			else {
				cerr << "Error by readying region data..." << endl;
				return 1;
			}
			for (int i=0; i<data.size(); i++)
				if (currentRegion.checkSMRegionMembership (data[i][0].featureVector))
					currentRegion.data.push_back (data[i]);
			
			currentRegion.generateCryssmexFiles (max_iterations, ssm/*, save_all*/, prefix, regionFileName, limits, featureSelectionMethod);
		}
	}

		
	return 0;
}
