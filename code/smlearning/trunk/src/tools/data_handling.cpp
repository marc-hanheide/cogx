/** @file data_handling.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa

   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.
 
*/

#include <tools/data_handling.h>


namespace smlearning {

///
///print DataSetParams tuple
///
void print_dataset_params (const DataSetParams& p) {
	cout << "motorVectorSize: " << p.get<lParams>().get<0>() << ",\n";
	cout << "featureVectorSize: " << p.get<lParams>().get<1>() << ",\n";
	cout << "pfVectorSize: " << p.get<lParams>().get<2>() << ",\n";
	cout << "efVectorSize: " << p.get<lParams>().get<3>() << ",\n";
	cout << "storeLabels: " << p.get<labels>() << "," << endl;
	cout << "minX: " << p.get<limits>().get<0>() << "," << endl;
	cout << "minY: " << p.get<limits>().get<1>() << "," << endl;
	cout << "minZ: " << p.get<limits>().get<2>() << "," << endl;
	cout << "maxX: " << p.get<limits>().get<3>() << "," << endl;
	cout << "maxY: " << p.get<limits>().get<4>() << "," << endl;
	cout << "maxZ: " << p.get<limits>().get<5>() << endl;
}


///
///write DataSet vector to a file
///
bool write_dataset (string fileName, const DataSetStruct& data) {
	fileName += ".seq";
	ofstream writeFile(fileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;

	//write data parameters
	writeFile.write((const char*)&data.second.get<lParams>().get<mVSize>(), sizeof(int));
	writeFile.write((const char*)&data.second.get<lParams>().get<fVSize>(), sizeof(int));
	writeFile.write((const char*)&data.second.get<lParams>().get<pfVSize>(), sizeof(int));
	writeFile.write((const char*)&data.second.get<lParams>().get<efVSize>(), sizeof(int));
	writeFile.write((const char*)&data.second.get<labels>(), sizeof(bool));
	writeFile.write((const char*)&data.second.get<limits>().get<minX>(), sizeof(Real));
	writeFile.write((const char*)&data.second.get<limits>().get<minY>(), sizeof(Real));
	writeFile.write((const char*)&data.second.get<limits>().get<minZ>(), sizeof(Real));
	writeFile.write((const char*)&data.second.get<limits>().get<maxX>(), sizeof(Real));
	writeFile.write((const char*)&data.second.get<limits>().get<maxY>(), sizeof(Real));
	writeFile.write((const char*)&data.second.get<limits>().get<maxZ>(), sizeof(Real));
	
	long numSeqs = data.first.size();
	writeFile.write ((const char*)&numSeqs, sizeof(numSeqs));
//  	cout << numSeqs << endl;
	DataSet::const_iterator s;
	for (s=data.first.begin(); s!= data.first.end(); s++) {
		long seqSize = s->size();
		writeFile.write ((const char*)&seqSize, sizeof (seqSize));
//  		cout << "\t" << seqSize << endl;
		Sequence::const_iterator v;

		for (v=s->begin(); v!= s->end(); v++) {
			write_vector<double> (writeFile, *v);
		}
	}
	
	writeFile.close();
	return true;
}



///
///read DataSet vector from a file
///
bool read_dataset (string fileName, DataSetStruct& dataStr) {
	fileName += ".seq";
	ifstream readFile(fileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;

	DataSet data;
	DataSetParams params;

	//get data parameters
	readFile.read((char*)&params.get<lParams>().get<mVSize>(), sizeof(int));
	readFile.read((char*)&params.get<lParams>().get<fVSize>(), sizeof(int));
	readFile.read((char*)&params.get<lParams>().get<pfVSize>(), sizeof(int));
	readFile.read((char*)&params.get<lParams>().get<efVSize>(), sizeof(int));
	readFile.read((char*)&params.get<labels>(), sizeof(bool));
	readFile.read((char*)&params.get<limits>().get<minX>(), sizeof(Real));
	readFile.read((char*)&params.get<limits>().get<minY>(), sizeof(Real));
	readFile.read((char*)&params.get<limits>().get<minZ>(), sizeof(Real));
	readFile.read((char*)&params.get<limits>().get<maxX>(), sizeof(Real));
	readFile.read((char*)&params.get<limits>().get<maxY>(), sizeof(Real));
	readFile.read((char*)&params.get<limits>().get<maxZ>(), sizeof(Real));

	long numSeq;
	readFile.read ((char* )&numSeq, sizeof(numSeq));
//  	cout << numSeq << endl;
	for (int s=0; s<numSeq; s++) {
		Sequence currentSequence;
		long seqSize;
		readFile.read((char *)&seqSize, sizeof(seqSize));
//  		cout << "\t" << seqSize << endl;
		for (int v=0; v<seqSize; v++) {
			FeatureVector currentVector;
			read_vector<double> (readFile, currentVector);
			currentSequence.push_back (currentVector);
		}
		data.push_back(currentSequence);
	}	

	dataStr = make_pair (data, params);
	readFile.close();
	return true;
}



/*bool check_nc_err(const int stat, const int line, const char *file) {
	if (stat != NC_NOERR) {
		cerr << "line" <<  line << "of" << file << ": " << nc_strerror(stat) << endl;
		return true;
	}
	return false;
}

///
///almost automatically generated netcdf function to store netcdf data files
///
bool write_nc_data (string fileName, const DataSet& data, int inputVectorSize, int targetVectorSize, FeatureVector& inputVector, FeatureVector& targetVector, vector<int>& seqLengthsVector, size_t& numTimesteps_len) {

	//return status
	int stat;
	//netCDF id
	int ncid;
	//dimension ids
	int numSeqs_dim;
	int numTimesteps_dim;
	int inputPattSize_dim;
	int targetPattSize_dim;
	//dimension lengths
	size_t numSeqs_len = data.size();
	size_t inputPattSize_len = inputVectorSize;
	size_t targetPattSize_len = targetVectorSize;
	//variable ids
	int inputs_id;
	int seqLengths_id;
	int targetPatterns_id;
	//rank for variables
	const int RANK_inputs = 2;
	const int RANK_seqLengths = 1;
	const int RANK_targetPatterns = 2;

	//enter define mode
	stat = nc_create(fileName.c_str(), NC_CLOBBER, &ncid);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;

	//define dimensions
	stat = nc_def_dim (ncid, "numSeqs", numSeqs_len, &numSeqs_dim);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;
	stat = nc_def_dim(ncid, "numTimesteps", numTimesteps_len, &numTimesteps_dim);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;
	stat = nc_def_dim(ncid, "inputPattSize", inputPattSize_len, &inputPattSize_dim);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;
	stat = nc_def_dim(ncid, "targetPattSize", targetPattSize_len, &targetPattSize_dim);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;

	//variable shapes
	int inputs_dims[RANK_inputs] = {numTimesteps_dim, inputPattSize_dim};
	int seqLengths_dims[RANK_seqLengths] = {numSeqs_dim};
	int targetPatterns_dims[RANK_targetPatterns] = {numTimesteps_dim, targetPattSize_dim};
	//define variables
	stat = nc_def_var(ncid, "inputs", NC_FLOAT, RANK_inputs, inputs_dims, &inputs_id);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;

	stat = nc_def_var(ncid, "seqLengths", NC_INT, RANK_seqLengths, seqLengths_dims, &seqLengths_id);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;

	stat = nc_def_var(ncid, "targetPatterns", NC_FLOAT, RANK_targetPatterns, targetPatterns_dims, &targetPatterns_id);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;
	//leave define mode
	stat = nc_enddef (ncid);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;

	cout << "numTimesteps: " << numTimesteps_len << endl << "inputPattSize: " << inputPattSize_len << endl << "targetPattSize: " << targetPattSize_len << endl;
	cout << "inputVector.size(): " << inputVector.size() << endl;
	cout << "targetVector.size(): " << targetVector.size() << endl;
	assert (numTimesteps_len * inputPattSize_len == inputVector.size ());
	assert (numTimesteps_len * targetPattSize_len == targetVector.size ());

	{ //store inputs
		//float inputs[numTimesteps_len * inputPattSize_len];
		float *inputs = new float [sizeof(float) * numTimesteps_len * inputPattSize_len];
		FeatureVector::iterator it = inputVector.begin();
		for (int i=0;  it != inputVector.end(); i++, it++)
			inputs[i] = *it;
		stat = nc_put_var_float(ncid, inputs_id, inputs);
		if (check_nc_err (stat, __LINE__, __FILE__))
			return false;
		delete inputs;
	}
	{ //store seqLengths
		int seqLengths[seqLengthsVector.size()];
		vector<int>::iterator it = seqLengthsVector.begin();
		for (int i=0; it != seqLengthsVector.end(); i++, it++)
			seqLengths[i] = *it;
		stat = nc_put_var_int(ncid, seqLengths_id, seqLengths);
		if (check_nc_err (stat, __LINE__, __FILE__))
			return false;
	}
	{ //store targetPatterns
		//float targetPatterns[numTimesteps_len * targetPattSize_len];
		float *targetPatterns = new float [sizeof(float) * numTimesteps_len * targetPattSize_len];
		FeatureVector::iterator it = targetVector.begin();
		for (int i=0; it != targetVector.end(); i++, it++)
			targetPatterns[i] = *it;
		stat = nc_put_var_float(ncid, targetPatterns_id, targetPatterns);
		if (check_nc_err (stat, __LINE__, __FILE__))
			return false;
		delete targetPatterns;
	}
	stat = nc_close(ncid);
	if (check_nc_err (stat, __LINE__, __FILE__))
		return false;

	return true;

}*/



void load_ef_pos_off(FeatureVector featVec, smlearning::FeatureVector& vec) {
	vec.push_back(featVec[0]);
	vec.push_back(featVec[1]);
	vec.push_back(featVec[2]);

//cout << vec.size() << endl;
//cout << featVec[0] << endl;
//cout << featVec[1] << endl;
//cout << featVec[2] << endl;
//cout << vec.back() << endl;
//cout << "------------------------" << endl;


}

void load_speed_and_angle_off(double speed, double angle, smlearning::FeatureVector& vec) {
		vec.push_back(speed);
		vec.push_back(angle);
//cout << vec.size() << endl;
//cout << speed << endl;
//cout << angle << endl;
//cout << vec.back() << endl;
//cout << "------------------------" << endl;

}

void load_feat_vector_off(FeatureVector featVec, smlearning::FeatureVector& vec){

	FeatureVector::const_iterator n;

	for (n=featVec.begin(); n!= featVec.end(); n++) {
		vec.push_back (*n);
	}
//cout << vec.size() << endl;
//cout << featVec.back() << endl;
//cout << vec.back() << endl;

}



bool write_nc_file_Markov (string fileName, const DataSet& data) {
//Markov
	fileName += ".nc";

	int initialVectorSize = data[0][0].size();
	int featureVectorSize = initialVectorSize + data[0][1].size();
	//int featureVectorSize = 18;

	FeatureVector inputVector;
	FeatureVector targetVector;
	vector<int> seqLengthsVector;
	size_t numTimesteps_len = 0;
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		size_t seqSize = (*s).size() - 1 - 1;  // we need to substract this additional 1,  since we don't write the motorComVec (there is one vector less)
		seqLengthsVector.push_back( seqSize );
		numTimesteps_len += seqSize;

		Sequence::const_iterator v;
		v = (*s).begin();
		double speed = (*v)[3];
		double angle = (*v)[4];
		v++;
		Sequence::const_iterator start = v;
		

		for (v/*=(*s).begin()*/; v!= (*s).end(); v++) {
			//put inputs and targetPatterns data
			if (v+1 != (*s).end()) {
//cout << "///////////////////////" << endl;
//cout << "INPUT" << endl;
//cout << (*v).size() << endl;
				load_ef_pos_off((*v), inputVector);
				load_speed_and_angle_off(speed, angle, inputVector);
				load_feat_vector_off((*v), inputVector);
//cout << "///////////////////////" << endl;
			}

			if (v != start) {
//cout << "///////////////////////" << endl;
//cout << "TARGET" << endl;
				load_ef_pos_off((*v), targetVector);
				load_speed_and_angle_off(speed, angle, targetVector);
				load_feat_vector_off((*v), targetVector);
//cout << "///////////////////////" << endl;
			}
				
		}
	}

	//netcdf file storing
	//if (!write_nc_data (fileName, data, featureVectorSize, featureVectorSize, inputVector, targetVector, seqLengthsVector, numTimesteps_len))
	//	return false;

	return true;

}

///
///write a netcdf nc file format for feature vectors using basis vectors
///
bool write_nc_file_basis (string fileName, const DataSetStruct& data) {
	fileName += ".nc";

	//int initialVectorSize = data.first[0][0].size();
	//int featureVectorSize = initialVectorSize + data.first[0][1].size();
	int inputSize = data.second.get<lParams>().get<mVSize>() + data.second.get<lParams>().get<fVSize>();
	int outputSize = data.second.get<lParams>().get<pfVSize>();
	int motorVectorSize = data.second.get<lParams>().get<mVSize>();
	int targetIndexStart = data.second.get<lParams>().get<efVSize>();

	FeatureVector inputVector;
	FeatureVector targetVector;
	vector<int> seqLengthsVector;
	size_t numTimesteps_len = 0;
	DataSet::const_iterator s;
	for (s=data.first.begin(); s!= data.first.end(); s++) {
		size_t seqSize = s->size() - 1;
		seqLengthsVector.push_back( seqSize );
		numTimesteps_len += seqSize;

		Sequence::const_iterator v;
		for (v=s->begin(); v!= s->end(); v++) {
			// FeatureVector::const_iterator n;
			//put inputs and targetPatterns data
			if (v+1 != s->end()) {
				if (v != s->begin()) {
					//zero padding
					for (int i=0; i<motorVectorSize; i++)
						inputVector.push_back (0.0);
					for (int i=0; i<inputSize - motorVectorSize; i++)
						inputVector.push_back (v->at(i));

					
				}
				else {
					for (int i=0; i<motorVectorSize; i++)
						//motor command info
						inputVector.push_back (v->at(i));
					// //zero padding
					//completing with next feature vector info
					for (int i=0; i<inputSize - motorVectorSize; i++)
						//inputVector.push_back (0.0);
						inputVector.push_back ((v+1)->at(i));
				}
			}
			
			if (v != s->begin()) {
				/*for (int i=0; i<motorVectorSize; i++)
				  targetVector.push_back (0.0);*/
				// for (n=v->begin() + data.first[0][1].size()/2; n!= v->end(); n++)
				for (int i=targetIndexStart; i<targetIndexStart+outputSize; i++)
					//targetVector.push_back (*n);
					targetVector.push_back (v->at(i));
			}
		}
	}

	//netcdf file storing
	if (!LearningData::write_nc_data (fileName, data.first.size(), inputSize, outputSize, inputVector, targetVector, seqLengthsVector, numTimesteps_len))
		return false;

	return true;
	
}


void load_ef_pos_act(FeatureVector featVec, vector<float>& vec, int& count) {

//int c = (*count);
	vec[count++] = featVec[0];
	vec[count++] = featVec[1];
	vec[count++] = featVec[2];	
//cout << (*count) << endl;
//cout << featVec[0] << endl;
//cout << featVec[1] << endl;
//cout << featVec[2] << endl;
//cout << vec[c++] << endl;
//cout << vec[c++] << endl;
//cout << vec[c] << endl;
//cout << "------------------------" << endl;

}


void load_speed_and_angle_act(double speed, double angle, vector<float>& vec, int& count) {

	vec[count++] = speed;
	vec[count++] = angle;
//cout << (*count) << endl;
//cout << speed << endl;
//cout << angle << endl;
//cout << vec[3] << endl;
//cout << vec[4] << endl;

//cout << "------------------------" << endl;

}

void load_feat_vector_act(FeatureVector featVec, vector<float>& vec, int& count){

	FeatureVector::const_iterator n;

	for (n=featVec.begin(); n!= featVec.end(); n++)
		vec[count++] = *n;


}

void load_sequence_Markov (vector<float>& inputVector, vector<float>& targetVector, Sequence s) {
//Markov
	int contInput = 0;
	int contTarget = 0;

	Sequence::const_iterator v = s.begin();
	double speed = v->at(3);
	double angle = v->at(4);
	v++;

	for (; v!= s.end(); v++) {
		if (v+1 != s.end()) {
//cout << "///////////////////////" << endl;
//cout << "INPUT" << endl;
//cout << (*v).size() << endl;
			//load_ef_pos_act((*v), inputVector, contInput);
			load_speed_and_angle_act(speed, angle, inputVector, contInput);
			load_feat_vector_act((*v), inputVector, contInput);
//cout << "///////////////////////" << endl;
		}

		if (v != s.begin() + 1) {
//cout << "///////////////////////" << endl;
//cout << "TARGET" << endl;
			//load_ef_pos_act((*v), targetVector, pContTarget);
			//load_speed_and_angle_act(speed, angle, targetVector, contTarget);
			//for (; contTarget<s.begin()->size(); contTarget++)
			//	targetVector[contTarget] = 0.0;
			load_feat_vector_act((*v), targetVector, contTarget);
//cout << "///////////////////////" << endl;
		}
	}

//cout << inputVector.size() << endl;
//for (int a = 0; a < 10; a++) {
//	cout << inputVector[a] << endl;
//}

}


///
///load a sequence into inputs and target vectors (for machine learning) (basis representation)
///
// void load_sequence_basis (vector<float>& inputVector, vector<float>& targetVector, Sequence s, int inputSize, int outputSize, int motorVectorSize, int targetIndexStart) {
void load_sequence_basis (vector<float>& inputVector, vector<float>& targetVector, Sequence s, DataSetParams params) {

	// int initialVectorSize = s[0].size();
	// int sensorVectorSize = s[1].size();
	// int featureVectorSize = initialVectorSize + sensorVectorSize;
	int inputSize = params.get<lParams>().get<mVSize>() + params.get<lParams>().get<fVSize>();
	int outputSize = params.get<lParams>().get<pfVSize>();
	int motorVectorSize = params.get<lParams>().get<mVSize>();
	int targetIndexStart = params.get<lParams>().get<efVSize>();
	
	Sequence::const_iterator v;
	int contInput = 0;
	int contTarget = 0;
	for (v=s.begin(); v!= s.end(); v++) {
		//FeatureVector::const_iterator n;
		//put inputs and targetPatterns data
		if (v+1 != s.end()) {
			if (v != s.begin()) {
				for (int i=0; i<motorVectorSize; i++)
					//zero padding
					inputVector[contInput++] = 0.0;
				for (int i=0; i<inputSize - motorVectorSize; i++)
					inputVector[contInput++] = v->at(i);
			}
			else {
				for (int i=0; i<motorVectorSize; i++)
					//motor command info
					inputVector[contInput++] = v->at(i);
				for (int i=0; i<inputSize - motorVectorSize; i++)
					// //zero padding
					//inputVector[contInput++] = 0.0;
					//completing with next feature vector info
					inputVector[contInput++] = (v+1)->at(i);
			}
		}
			
		if (v != s.begin()) {
			/*for (int i=0; i<motorVectorSize; i++)
			  targetVector[contTarget++] = 0.0;*/
			// for (n=v->begin() + sensorVectorSize/2; n!= v->end(); n++) {
			for (int i=targetIndexStart; i<targetIndexStart+outputSize; i++)
				//targetVector[contTarget++] = *n;
				targetVector[contTarget++] = v->at(i);

		}
	}

	
}


//------------------------------------------------------------------------------
///
///load training data in RNNLIB format
///
rnnlib::DataSequence* load_trainSeq (smlearning::Sequence& seq, DataSetParams params) {
	int inputSize = params.get<lParams>().get<mVSize>() + params.get<lParams>().get<fVSize>();
	int outputSize = params.get<lParams>().get<pfVSize>();
	rnnlib::DataSequence* trainSeq = new rnnlib::DataSequence (inputSize, outputSize);
	vector<int> inputShape, targetShape;
	inputShape.push_back (seq.size() - 1);
	targetShape.push_back (seq.size() - 1);
	trainSeq->inputs.reshape(inputShape);
	trainSeq->targetPatterns.reshape(targetShape);
	load_sequence_basis (trainSeq->inputs.data, trainSeq->targetPatterns.data, seq, params/*inputSize, outputSize, motorVectorSize, targetIndexStart*/);
	//load_sequence_Markov (trainSeq->inputs.data, trainSeq->targetPatterns.data, seq);
	return trainSeq;

}

//------------------------------------------------------------------------------
bool check_params (DataSetParams params1, DataSetParams params2) {
	if (params1.get<lParams>().get<mVSize>() != params2.get<lParams>().get<mVSize>())
		return false;
	if (params1.get<lParams>().get<fVSize>() != params2.get<lParams>().get<fVSize>())
		return false;
	if (params1.get<lParams>().get<pfVSize>() != params2.get<lParams>().get<pfVSize>())
		return false;
	if (params1.get<lParams>().get<efVSize>() != params2.get<lParams>().get<efVSize>())
		return false;
	if (params1.get<labels>() != params2.get<labels>())
		return false;
	if (params1.get<limits>().get<minX>() != params2.get<limits>().get<minX>())
		return false;
	if (params1.get<limits>().get<minY>() != params2.get<limits>().get<minY>())
		return false;
	if (params1.get<limits>().get<minZ>() != params2.get<limits>().get<minZ>())
		return false;
	if (params1.get<limits>().get<maxX>() != params2.get<limits>().get<maxX>())
		return false;
	if (params1.get<limits>().get<maxY>() != params2.get<limits>().get<maxY>())
		return false;
	if (params1.get<limits>().get<maxZ>() != params2.get<limits>().get<maxZ>())
		return false;
	return true;
}

//------------------------------------------------------------------------------

bool concatenate_datasets (string dir, string writeFileName) {
	boost::regex seqfile_re ("(.*)\\.seq");
	boost::cmatch matches;
	path p(dir);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		return false;
	}

	directory_iterator dir_iter(p), dir_end;
	DataSetStruct data;
	bool checkFlag = false;
	for(;dir_iter != dir_end; ++dir_iter) {
		string dirstring (dir_iter->leaf().c_str());
		char *dirchar = (char *)dirstring.c_str();

		if (boost::regex_match((const char*)dirchar, matches, seqfile_re)) {
			DataSetStruct currentData;
			string dataBaseName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << dataBaseName << endl;
			read_dataset (dataBaseName, currentData);
			if (!checkFlag) {
				data.second = currentData.second;
				checkFlag = true;
			}
			else {
				assert (check_params (data.second, currentData.second));
			}

			cout << "size current data: " << currentData.first.size() << endl;
			DataSet::iterator it = data.first.end();
			data.first.insert (it, currentData.first.begin(), currentData.first.end());
		}
	}
	cout << "size data: " << data.first.size() << endl;
	if (!write_dataset (writeFileName, data) ) {
		cerr << "Error writing dataset file!" << endl;
		return false;
	}
	
	return true;
}
	

///
///returns file name based on current time
///
string get_base_filename_from_time () {
	time_t rawtime;
	struct tm * timeinfo;
  	char buffer [12];

 	time ( &rawtime );
  	timeinfo = localtime ( &rawtime );

  	strftime (buffer,12,"%y%m%d%H%M",timeinfo);
  	puts(buffer);

	string name;
	name.append(buffer);

	return name;

}


///
///a utility function to obtain the file name from a possibly large path/file pattern
///
string get_seqBaseFileName (string seqFile) {
	boost::regex seqfile_re (".*/(.*)$");
	boost::cmatch match;
	string seqBaseFileName;
	if (boost::regex_match(seqFile.c_str(), match, seqfile_re)) {
		seqBaseFileName = string(match[1].first, match[1].second);
	}
	else
		seqBaseFileName = seqFile;
	cout << "seqBaseFileName: " << seqBaseFileName << endl;
	cout << "seqFile: " << seqFile << endl;
	return seqBaseFileName;
}

///
///obtain a discretization of starting finger poses from a canonical set of actions
///
map<Vec3, int, compare_Vec3> get_canonical_positions (Scenario::Desc& desc) {
	//generate all possible polyflap poses
	//TODO: the following data should be obtained from an xml file

	//Polyflap Position and orientation
	/*const Vec3 startPolyflapPosition(Real(0.2), Real(0.2), Real(0.0));
	const Vec3 startPolyflapRotation(Real(0.0*REAL_PI), Real(0.0*REAL_PI), Real(0.0*REAL_PI));//Y,X,Z
	//Polyflap dimensions		
	const Vec3 polyflapDimensions(Real(0.1), Real(0.1), Real(0.1)); //w,h,l
		

// 	//vertical distance from the ground
// 	const Real over = 0.01;
	//vertical distance from the ground considering fingertip radius
	Real over = 0.002 + 0.015;
	//distance from the front/back of the polyflap
	const Real dist = 0.05;
	//distance from the side of the polyflap
	const Real side = polyflapDimensions.v1*0.6;
	//center of the polyflap
	const Real center = polyflapDimensions.v2*0.6;
	//distance from the top of the polyflap
	//const Real top = polyflapDimensions.v2* 1.2;
	const Real top = polyflapDimensions.v2 - 0.02;
	//lenght of the movement		
	const Real distance = 0.2;*/

	//initialization of arm target: the center of the polyflap
	//Vec3 positionT (0.2, 0.2, 0.001);
	Vec3 positionT (desc.startPolyflapPosition.v1, desc.startPolyflapPosition.v2, desc.pfWidth * 0.5);
	//Normal vector showing the direction of the lying part of polyflap, and it' orthogonal
	//Vec3 polyflapNormalVec = computeNormalVector(Vec3 (0.2, 0.2, 0.0),Vec3 (0.2, 0.25, 0.0));
	Vec3 standingPolyflapPosition (desc.startPolyflapPosition.v1, desc.startPolyflapPosition.v2 + desc.polyflapDimensions.v2 * 0.5, desc.startPolyflapPosition.v3);
	cout << "standingPolPos: " << standingPolyflapPosition.v1 << "," << standingPolyflapPosition.v2 << "," << standingPolyflapPosition.v3 << endl;
	Vec3 polyflapNormalVec = computeNormalVector(desc.startPolyflapPosition, standingPolyflapPosition);
	
	Vec3 polyflapOrthogonalVec = computeOrthogonalVec(polyflapNormalVec);

	map<Vec3, int, compare_Vec3> positionsT;

	vector<int> availableStartingPositions = parse_startingPositions(desc.startingPositionsConfig, Scenario::startingPositionsCount);


	//int smRegionsCount = 18;
	//for (int i=1; i <= smRegionsCount; i++) {
	for (int i=0; i<availableStartingPositions.size(); i++) {
		//arm target update
		
		Vec3 pos (positionT);
		///*Scenario::*/set_coordinates_into_target(i, pos, polyflapNormalVec, polyflapOrthogonalVec, dist, side, center, top, over);
		set_coordinates_into_target(availableStartingPositions[i], pos, polyflapNormalVec, polyflapOrthogonalVec, desc.dist, desc.side, desc.center, desc.top, desc.over);
		positionsT[pos] = availableStartingPositions[i];
		
	}

	return positionsT;
}

///
///enumerate a dataset taking into account time steps
///
CanonicalData::DataSetStruct canonical_input_output_enumerator_with_time (DataSetStruct& data, Scenario::Desc& desc, int modulo) {
	
	//a number that slightly greater then the maximal reachable space of the arm
	//    - used for workspace position normalization and later as a position upper bound
	//      for random polyflap position
	//Real maxRange = 0.4;
	assert (data.second.get<limits>().get<minX>() == desc.coordLimits.minX);
	assert (data.second.get<limits>().get<minY>() == desc.coordLimits.minY);
	assert (data.second.get<limits>().get<minZ>() == desc.coordLimits.minZ);
	assert (data.second.get<limits>().get<maxX>() == desc.coordLimits.maxX);
	assert (data.second.get<limits>().get<maxY>() == desc.coordLimits.maxY);
	assert (data.second.get<limits>().get<maxZ>() == desc.coordLimits.maxZ);
	assert (data.second.get<labels>());
	assert (data.second.get<lParams>().get<mVSize>() == 5);

	map<Vec3, int, compare_Vec3> positionsT = get_canonical_positions (desc);
	
	map<Vec3, int>::const_iterator it;
	cout << "map size: " << positionsT.size() << endl;
	for (it = positionsT.begin(); it != positionsT.end(); it++) {
		cout << "canon. pos.: "  << it->second  << ": stored start. pos.: "  << it->first.v1 << " " << it->first.v2 << " " << it->first.v3 << endl;
	}

	//Construct a new data set using a simple artificial discretization
	//A canonical set of starting positions are obtained (i.e. the 18)
	//Depending on feature vector size, polyflap poses and eventually finger
	//effector poses are extracted
	CanonicalData::DataSet newdata;
	DataSet::const_iterator s;
	for (s=data.first.begin(); s!= data.first.end(); s++) {
		Sequence::const_iterator v;
		CanonicalData::Sequence newsequence;

		stringstream motorCommandBaseStr;
		int time_step = 0;
		for (v=s->begin(); v!= s->end(); v++) {
			stringstream motorCommandStr;
			CanonicalData::FeatureVector newfeaturevector;
			long featvectorSize = v->size();
			if (v == s->begin() ) {
				//assert (featvectorSize == 5);
				golem::Vec3 startingPosition;
				startingPosition.v1 = denormalize(v->at(0),desc.coordLimits.minX,desc.coordLimits.maxX);
				startingPosition.v2 = denormalize(v->at(1),desc.coordLimits.minY,desc.coordLimits.maxY);
				startingPosition.v3 = denormalize(v->at(2),desc.coordLimits.minZ,desc.coordLimits.maxZ);
// 				printf ("extracted start. pos.: %0.20f %0.20f %0.20f\n", startingPosition.v1, startingPosition.v2, startingPosition.v3);
				
				int canonical_start_pos = positionsT.find (startingPosition)->second;
				newfeaturevector.rawVector.push_back (canonical_start_pos);
				newfeaturevector.rawVector.push_back (v->at(3));
				newfeaturevector.rawVector.push_back (v->at(4));
// 				stringstream motorCommandStr;
				motorCommandBaseStr << canonical_start_pos << "_" << v->at(3) << "_" << v->at(4);
				newfeaturevector.motorCommand = motorCommandBaseStr.str();
// 				cout << "motor command: " << newfeaturevector.motorCommand << endl;
				newsequence.push_back (newfeaturevector);
			}
			else if (v != s->begin() && time_step % modulo == 0) {
				/*assert (featvectorSize == 13 || featvectorSize == 7 );
				if (featvectorSize == 7)
					for (int i=0; i<6; i++)
						newfeaturevector.rawVector.push_back(v->at(i));
				else if (featvectorSize == 13)
					for (int i=6; i<12; i++)
					newfeaturevector.rawVector.push_back(v->at(i));*/
				for (int i=0; i<featvectorSize-1; i++)
					newfeaturevector.rawVector.push_back(v->at(i));
				stringstream labelStr;
				labelStr << v->at(featvectorSize-1);
				newfeaturevector.label = labelStr.str();
				motorCommandStr << motorCommandBaseStr.str() << "_" << time_step;
				newfeaturevector.motorCommand = motorCommandStr.str();
				newsequence.push_back (newfeaturevector);
			
			}
			// newsequence.push_back (newfeaturevector);
			time_step++;
		}
		newdata.push_back (newsequence);
	}
	cout << endl;

	return make_pair(newdata, data.second);
}


///
///write a dataset in cryssmex format by using the canonical representation
///
void write_canonical_dataset_cryssmex_fmt (string writeFileName, CanonicalData::DataSet data) {
	//assuming an input(output) vector be symbolic (dimensionality 0 in cryssmex)
	writeFileName += ".cry";

	assert (data.size() >= 1);
	assert (data[0].size() >= 2);
	
	int inputVectorSize = 0;
	int outputVectorSize = 0;
	int stateVectorSize = data[0][1].rawVector.size();
	
	ofstream writeFile(writeFileName.c_str(), ios::out | ios::binary);

	writeFile << "# input dim" << endl;
	writeFile << inputVectorSize << endl;
	writeFile << "# state dim" << endl;
	writeFile << stateVectorSize << endl;
	writeFile << "# output dim" << endl;
	writeFile << outputVectorSize << endl;

	set<string> motorCommandsSet;
	set<string> outputsSet;
	CanonicalData::DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {

		CanonicalData::Sequence::const_iterator v;
		for (v=s->begin(); v!= s->end(); v++) {
			if (v != s->begin()) {
				string currentMotorCommand = v->motorCommand;
				if (motorCommandsSet.find (currentMotorCommand) == motorCommandsSet.end())
					motorCommandsSet.insert (currentMotorCommand);
				string currentOutput = v->label;
				
				if (outputsSet.find (currentOutput) == outputsSet.end())
					outputsSet.insert (currentOutput);
			}
		}
	}
	writeFile << "# nr input symbols" << endl;
	writeFile << motorCommandsSet.size() << endl;
	writeFile << "# examples" << endl;
	set<string>::iterator it;
	for (it=motorCommandsSet.begin(); it!=motorCommandsSet.end(); it++)
		writeFile << *it << " ";
	writeFile << endl;
	writeFile << "# nr output symbols" << endl;
	writeFile << outputsSet.size() << endl;
	writeFile << "# examples" << endl;
	for (it=outputsSet.begin(); it!=outputsSet.end(); it++)
		writeFile << *it << " ";
	writeFile << endl;
	writeFile.precision(20);

	//CanonicalData::DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		CanonicalData::Sequence::const_iterator v;

		for (v=s->begin(); v!= s->end(); v++) {
			
			if (v != s->begin()) {
				FeatureVector::const_iterator n;
				string currentMotorCommand = v->motorCommand;
				writeFile << currentMotorCommand << "   ";
				for (n=v->rawVector.begin(); n!= v->rawVector.end(); n++)
					writeFile << *n << " ";
				writeFile << "  " << v->label << endl;
			}
		}
	}
	writeFile.close();
}

///
///write a dataset in cryssmex format. This code assumes vectorial data format and an output
///label defined artificially
///
void write_dataset_cryssmex_fmt_with_label (string writeFileName, DataSet data, int modulo) {

	//this function assumes data coming from feature vectors in dataset
	//the first vector has the motor command information (of size 5 is for now assumed)
	//the rest of vectors contain a 6D finger effector pose,
	//a 6D polyflap pose and a final output value in that order
	int inputVectorSize;
	assert ((inputVectorSize = data[0][0].size() + (data[0][1].size()-1)/2) == 11);
	int stateVectorSize;
	assert ((stateVectorSize = (data[0][1].size() - 1) /2) == 6);
	int outputVectorSize = 1;

	writeFileName += ".cry";
	ofstream writeFile(writeFileName.c_str(), ios::out | ios::binary);

	writeFile << "# input dim" << endl;
	writeFile << inputVectorSize << endl;
	writeFile << "# state dim" << endl;
	writeFile << stateVectorSize << endl;
	writeFile << "# output dim" << endl;
	writeFile << outputVectorSize << endl;
	writeFile << "# input examples" << endl << "0.0" << endl;
	writeFile << "# output examples" << endl << "0.0" << endl;
	writeFile.precision(20);
	
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		Sequence::const_iterator v;

		FeatureVector motorCommandVector;
		int time_step = 0;
		for (v=s->begin(); v!= s->end(); v++) {
			FeatureVector::const_iterator n;
			long featvectorSize = v->size();
			if (v == s->begin() )
				for (n=v->begin(); n!= v->end(); n++)
					motorCommandVector.push_back (*n);
			else if (v != s->begin() && time_step % modulo == 0) {
				for (n=motorCommandVector.begin(); n!=motorCommandVector.end(); n++)
					writeFile << *n << "  ";
				for (n=v->begin(); n!= v->end()-1; n++)
					writeFile << *n << "  ";
				writeFile << *(v->end()-1) << endl;
			}
			time_step++;
		}
	}
	writeFile.close();
}

///
///write a dataset in cryssmex format. This code assumes vectorial data format
///and works as a regression predictor and canonical format for the input
///
void write_canonical_dataset_cryssmex_fmt_regression (string writeFileName, CanonicalData::DataSetStruct& data, bool pfefState, bool pfefOutput ) {
	//assuming an input vector be symbolic (dimensionality 0 in cryssmex)
	
	writeFileName += ".cry";

	assert (data.first.size() >= 1);
	assert (data.first[0].size() >= 2);
	
	int inputVectorSize = 0;
	int stateVectorSize;
	int outputVectorSize;
	if (pfefState)
		stateVectorSize = data.second.get<lParams>().get<fVSize>();
	else
		stateVectorSize = data.second.get<lParams>().get<pfVSize>();
	if (pfefOutput)
		outputVectorSize = data.second.get<lParams>().get<fVSize>();
	else
		outputVectorSize = data.second.get<lParams>().get<pfVSize>();

	ofstream writeFile(writeFileName.c_str(), ios::out | ios::binary);

	writeFile << "# input dim" << endl;
	writeFile << inputVectorSize << endl;
	writeFile << "# state dim" << endl;
	writeFile << stateVectorSize << endl;
	writeFile << "# output dim" << endl;
	writeFile << outputVectorSize << endl;

	set<string> motorCommandsSet;
	CanonicalData::DataSet::const_iterator s;
	for (s=data.first.begin(); s!= data.first.end(); s++) {

		CanonicalData::Sequence::const_iterator v;
		for (v=s->begin(); v!= s->end(); v++) {
			if (v != s->begin()) {
				string currentMotorCommand = v->motorCommand;
				if (motorCommandsSet.find (currentMotorCommand) == motorCommandsSet.end())
					motorCommandsSet.insert (currentMotorCommand);
			}
		}
	}
	writeFile << "# nr input symbols" << endl;
	writeFile << motorCommandsSet.size() << endl;
	writeFile << "# examples" << endl;
	set<string>::iterator it;
	for (it=motorCommandsSet.begin(); it!=motorCommandsSet.end(); it++)
		writeFile << *it << " ";
	writeFile << endl;
	writeFile << "# output examples" << endl << "0.0" << endl;
	// writeFile.precision(20);

	for (s=data.first.begin(); s!= data.first.end(); s++) {
		CanonicalData::Sequence::const_iterator v;

		for (v=s->begin(); v!= s->end(); v++) {
			
			if (v != s->begin() && v+1 != s->end()) {
				FeatureVector::const_iterator n;
				string currentMotorCommand = v->motorCommand;
				writeFile << currentMotorCommand << "   ";
				int start;
				pfefState ? start=0 : start=data.second.get<lParams>().get<efVSize>();
				for (int i=start; i<stateVectorSize+start; i++)
					writeFile << v->rawVector[i] << " ";
				// for (n=v->rawVector.begin(); n!= v->rawVector.end(); n++)
				//	writeFile << *n << " ";
				
				pfefOutput ? start=0 : start=data.second.get<lParams>().get<efVSize>();
				for (int i=start; i<outputVectorSize+start; i++)
					writeFile << (v+1)->rawVector[i] << " ";
				// for (n=(v+1)->rawVector.begin(); n!=(v+1)->rawVector.end(); n++)
				// 	writeFile << *n << " ";
				writeFile << endl;
			}
		}
	}
	writeFile.close();


}

///
///write a dataset in cryssmex format. This code assumes vectorial data format
///and works as a regression predictor
///\params pfEfPredictor variable allows prediction of both polyflap and effector vectors
///
void write_dataset_cryssmex_fmt_regression (string writeFileName, DataSetStruct& data, bool pfefState = false, bool pfefOutput = false) {

	writeFileName += ".cry";

	assert (data.first.size() >= 1);
	assert (data.first[0].size() >= 2);

	int inputVectorSize = data.second.get<lParams>().get<mVSize>();;
	int stateVectorSize;
	int outputVectorSize;
	if (pfefState)
		stateVectorSize = data.second.get<lParams>().get<fVSize>();
	else
		stateVectorSize = data.second.get<lParams>().get<pfVSize>();
	if (pfefOutput)
		outputVectorSize = data.second.get<lParams>().get<fVSize>();
	else
		outputVectorSize = data.second.get<lParams>().get<pfVSize>();

	ofstream writeFile(writeFileName.c_str(), ios::out | ios::binary);

	writeFile << "# input dim" << endl;
	writeFile << inputVectorSize << endl;
	writeFile << "# state dim" << endl;
	writeFile << stateVectorSize << endl;
	writeFile << "# output dim" << endl;
	writeFile << outputVectorSize << endl;

	writeFile << "# nr input symbols" << endl << "0.0" << endl;
	writeFile << "# output examples" << endl << "0.0" << endl;

	DataSet::const_iterator s_iter;

	for (s_iter=data.first.begin(); s_iter!= data.first.end(); s_iter++) {
		Sequence::const_iterator v_iter;

		if (v_iter == s_iter->begin()) {
		}
	}

		
}


///
///parse a string containing a list of starting positions (TODO: write a nice reg.exp. :P )
///
vector<int> parse_startingPositions(string argsStr, int maxStartPos) {
	
	vector<int> positions;
	int currNumber1 = 0;
	int currNumber2 = 0;
	bool range = false;



	string::iterator it;
	for ( it=argsStr.begin() ; it < argsStr.end(); it++ ) {
		const char character = *it;
		// cout << "character "  << character << endl;
		if (isdigit(character) && !range) {
			currNumber1 *= 10;
			currNumber1 += atoi(&character);
			// cout << "case 1: new nr1: " << currNumber1 << endl;
		} else if (isdigit(character) && range) {
			currNumber2 *= 10;
			currNumber2 += atoi(&character);
			// cout << "case 2: new nr2: " << currNumber2 << endl;
		} else if (character == "-"[0]) {
			// cout << "case 3: dash" << endl;
			range = true;
		} else  if (character == ","[0]) {
			// cout << "case 4: comma" << endl;
			if (currNumber1 > 0 && currNumber1 <= maxStartPos) {
				// cout << "appending nr1 :" << currNumber1 << endl;
			positions.push_back(currNumber1);
			}
			if (range) {
				// cout << "range on" << endl;
				for (int i = (currNumber1 + 1); i < currNumber2; i++) {
					if (i > 0 && i <= maxStartPos) {
					positions.push_back(i);
					// cout << "appending :" << i << endl;
					}
				}
				if (currNumber1 > 0 && currNumber2 <= maxStartPos) {
					// cout << "appending nr2 :" << currNumber2 << endl;
				positions.push_back(currNumber2);			
				}
			}
			currNumber1 = 0;
			currNumber2 = 0;
			range = false;
// cout << "variables set back: " << currNumber1 << " " << currNumber2 << " " << range << endl;
		} else {
			// cout << "nothing" << endl;
		} 
	}
	if (argsStr.at(argsStr.length()-1) != ","[0]) {
			if (currNumber1 > 0 && currNumber1 <= maxStartPos) {
				// cout << "appending nr1 :" << currNumber1 << endl;
			positions.push_back(currNumber1);
			}
			if (range) {
				// cout << "range on" << endl;
				for (int i = (currNumber1 + 1); i < currNumber2; i++) {
					if (i > 0 && i <= maxStartPos) {
					positions.push_back(i);
					// cout << "appending :" << i << endl;
					}
				}
				if (currNumber1 > 0 && currNumber2 <= maxStartPos) {
					// cout << "appending nr2 :" << currNumber2 << endl;
				positions.push_back(currNumber2);			
				}
			}
	}
	// cout << "finished" << endl;
	return positions;
}




};  /* smlearning namespace */
