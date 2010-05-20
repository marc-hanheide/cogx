#include <metalearning/SMRegion.h>

using namespace smlearning;

bool evaluate_activelearn (string seqFile, string dir) {

	DataSet data;

	if (!read_dataset (seqFile, data)) {
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


	map<int, SMRegion> regions;
	
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
		int regionidx = SMRegion::get_SMRegion (regions, data[i][0]);
		assert (regionidx != -1);
		rnnlib::DataSequence* testSeq = load_trainSeq (data[i], SMRegion::motorVectorSize + SMRegion::featureVectorSize, SMRegion::pfVectorSize);
		error += regions[regionidx].learner.net->calculate_errors (*testSeq);
	}
	error /= (double)data.size();

	cout << "Avg. sum of squares error: " << error << endl;

 	return 0;
}


int main (int argc, char *argv[]) {

	string dir;
	string seqFile;
	if (argc >= 3) {
		dir = string (argv[2]);
	}
	if (argc >= 2) {
		dir = "./";
		seqFile = string (argv[1]);
	}
	else {
		cerr << argv[0] << " sequence_file (without extension) [dir (default:current dir.)]" << endl;
		return 1;
	}



	return evaluate_activelearn (seqFile, dir);


	
}
