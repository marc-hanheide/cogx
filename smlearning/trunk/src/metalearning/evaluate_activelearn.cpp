#include <metalearning/data_structs.h>
#include <metalearning/SMRegion.h>

using namespace smlearning;

bool evaluate_activelearn (string seqFile, string dir, unsigned int featureSelectionMethod) {

	LearningData::DataSet data;
	LearningData:: CoordinateLimits limits;

	if (!LearningData::read_dataset (seqFile, data, limits)) {
		cerr << "error reading data" << endl;
		return 1;
	}

	boost::regex regfile_re ("(.*_final)\\.reg");
	boost::cmatch matches;
	cout << matches.size() << endl;
	path p(dir);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		return 1;
	}


	SMRegion::RegionsMap regions;
	
	directory_iterator dir_iter(p), dir_end;
	for(;dir_iter != dir_end; ++dir_iter) {
		string dirstring (dir_iter->leaf().c_str());
		char *dirchar = (char *)dirstring.c_str();
		if (boost::regex_match((const char*)dirchar, matches, regfile_re)) {
		
			SMRegion currentRegion;
			string regionFileName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << regionFileName << endl;
			if (currentRegion.read_data (regionFileName)) {
				cout << "region data correctly read..." << endl << "======" << endl;
				regions[currentRegion.index] = currentRegion;
			}
		}
	}

	double error = 0.0;
	for (int i = 0; i < data.size(); i++) {
		FeatureVector featureVector;
		LearningData::write_chunk_to_featvector (featureVector, data[i][0], normalize<double>, limits, _effector | _action_params);
		int regionidx = SMRegion::get_SMRegion (regions, featureVector);
		assert (regionidx != -1);
		rnnlib::DataSequence* testSeq = LearningData::load_NNtrainSeq (data[i], featureSelectionMethod, normalize<double>, limits );
		error += regions[regionidx].learner.net->calculate_errors (*testSeq);
	}
	error /= (double)data.size();

	cout << "Avg. sum of squares error: " << error << endl;

 	return 0;
}


int main (int argc, char *argv[]) {

	string dir;
	string seqFile;
	unsigned int featureSelectionMethod;
	if (argc >= 3) {
		dir = "./";
		seqFile = string (argv[1]);
		string fSMethod = string (argv[2]);
		if (fSMethod == "basis")
			featureSelectionMethod = _basis;
		else if (fSMethod == "markov")
			featureSelectionMethod = _markov;			
	}
	if (argc >= 4) {
		dir = string (argv[3]);
	}
	else {
		cerr << argv[0] << " sequence_file (without extension) basis/markov [dir (default:current dir.)]" << endl;
		return 1;
	}



	return evaluate_activelearn (seqFile, dir, featureSelectionMethod);


	
}
