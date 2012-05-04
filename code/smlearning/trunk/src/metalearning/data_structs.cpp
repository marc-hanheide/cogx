/** @file data_structs.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2010      Sergio Roa
 
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

#include <metalearning/data_structs.h>


namespace smlearning {

std::ostream& operator<<(std::ostream& _out, feature_selection _e) {
	return feature_selection_binder().print_enum(_out, _e);
}
  
std::istream& operator>>(std::istream& _in, feature_selection& _e) {
	return feature_selection_binder().read_enum(_in, _e);
}

///
///Write DataSet vector to a file
///
bool LearningData::write_dataset (string fileName, const DataSet& data, const FeaturesLimits& limits) {
	fileName += ".seq";
	ofstream writeFile(fileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;

	//write data parameters
	writeFile.write((const char*)&limits.minX, sizeof(Real));
	writeFile.write((const char*)&limits.minY, sizeof(Real));
	writeFile.write((const char*)&limits.minZ, sizeof(Real));
	writeFile.write((const char*)&limits.maxX, sizeof(Real));
	writeFile.write((const char*)&limits.maxY, sizeof(Real));
	writeFile.write((const char*)&limits.maxZ, sizeof(Real));
	writeFile.write((const char*)&limits.minDuration, sizeof(Real));
	writeFile.write((const char*)&limits.maxDuration, sizeof(Real));
	writeFile.write((const char*)&limits.minValLabel, sizeof(Real));
	writeFile.write((const char*)&limits.maxValLabel, sizeof(Real));

	long numSeqs = data.size();
	writeFile.write ((const char*)&numSeqs, sizeof(numSeqs));
	cout << numSeqs << " sequences." << endl;
	DataSet::const_iterator d_iter;
	for (d_iter=data.begin(); d_iter!=data.end(); d_iter++) {
		Chunk::Seq seq = *d_iter;
		long seqSize = seq.size();
		writeFile.write ((const char*)&seqSize, sizeof (seqSize));
  		// cout << "\t" << seqSize << endl;
		Chunk::Seq::const_iterator s_iter;
		for (s_iter=seq.begin(); s_iter != seq.end(); s_iter++) {
			Chunk currentChunk = *s_iter;
			writeFile.write ((const char*)&(currentChunk.object), sizeof(currentChunk.object));
			writeFile.write ((const char*)&(currentChunk.action), sizeof(currentChunk.action));
			writeFile.write ((const char*)&(currentChunk.label), sizeof(currentChunk.label));
			
		}

		
	}
	return true;
}


///
///Read DataSet vector from a file
///
bool LearningData::read_dataset (string fileName, DataSet& data, FeaturesLimits& limits) {
	fileName  += ".seq";
	ifstream readFile(fileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;

	//read data parameters
	readFile.read((char*)&limits.minX, sizeof(Real));
	readFile.read((char*)&limits.minY, sizeof(Real));
	readFile.read((char*)&limits.minZ, sizeof(Real));
	readFile.read((char*)&limits.maxX, sizeof(Real));
	readFile.read((char*)&limits.maxY, sizeof(Real));
	readFile.read((char*)&limits.maxZ, sizeof(Real));
	readFile.read((char*)&limits.minDuration, sizeof(Real));
	readFile.read((char*)&limits.maxDuration, sizeof(Real));
	readFile.read((char*)&limits.minValLabel, sizeof(Real));
	readFile.read((char*)&limits.maxValLabel, sizeof(Real));
	
	long numSeq;
	readFile.read ((char*)&numSeq, sizeof(numSeq));
// 	cout << "Nr. of seq.: " << numSeq << endl;

	for (int s=0; s<numSeq; s++) {
		Chunk::Seq currentChunkSeq;

		long seqSize;
		readFile.read((char *)&seqSize, sizeof(seqSize));
// 		cout << "  Seq. size: " << seqSize << endl;
		for (int c=0; c<seqSize; c++) {
			Chunk currentChunk;
			readFile.read((char *)&currentChunk.object, sizeof(currentChunk.object));
			readFile.read((char *)&currentChunk.action, sizeof(currentChunk.action));
			readFile.read((char *)&currentChunk.label, sizeof(currentChunk.label));
// 			print_Chunk (currentChunk);
			currentChunkSeq.push_back (currentChunk);
		}
		data.push_back (currentChunkSeq);
	}

	return true;	

}

///
///print a chunk
///
void LearningData::print_Chunk (const Chunk& c) {
	cout << "\tC=[";
	cout << " pD=" << c.action.pushDuration << ", ";
	cout << " hA=" << c.action.horizontalAngle << ", ";
	cout << " eP=" << c.action.effectorPose.p.v1 << " "
	     << c.action.effectorPose.p.v2 << " " << c.action.effectorPose.p.v3 << ",";
	cout << " eO=" << c.action.efRoll << " " << c.action.efPitch << " " << c.action.efYaw << ",";
	cout << " eeP=" << c.action.endEffectorPose.p.v1 << " "
	     << c.action.endEffectorPose.p.v2 << " " << c.action.endEffectorPose.p.v3 << ",";
	cout << " eeO=" << c.action.endEfRoll << " " << c.action.endEfPitch << " " << c.action.endEfYaw << ",";
	cout << " oP=" << c.object.objectPose.p.v1 << " "
	     << c.object.objectPose.p.v2 << " " << c.object.objectPose.p.v3 << ",";
	cout << " oO=" << c.object.obRoll << " " << c.object.obPitch << " " << c.object.obYaw << ", ";
	cout << " l=" << c.label << " ]" << endl;
}

///
///print coordinate limits for a dataset
///
void LearningData::print_dataset_limits (const FeaturesLimits limits) {
	cout << "Dataset limits: " << endl;
	cout << "min X = " << limits.minX << endl;
	cout << "min Y = " << limits.minY << endl;
	cout << "min Z = " << limits.minZ << endl;
	cout << "max X = " << limits.maxX << endl;
	cout << "max Y = " << limits.maxY << endl;
	cout << "max Z = " << limits.maxZ << endl;
	cout << "min Push Duration = " << limits.minDuration << endl;
	cout << "max Push Duration = " << limits.maxDuration << endl;
	cout << "min Label Value = " << limits.minValLabel << endl;
	cout << "max Label Value = " << limits.maxValLabel << endl;
}


///
///print a DataSet vector
///
void LearningData::print_dataset (const DataSet &d) {
	cout << "Dataset size: " << d.size() << endl;
	
	DataSet::const_iterator d_iter;

	for (d_iter = d.begin(); d_iter != d.end(); d_iter++) {
		//MotorCommand mC = d_iter->first;
		cout << "{";

		Chunk::Seq seq = *d_iter;

		cout << "\tSeq (size=" << seq.size() << ")" << "=(\n";

		Chunk::Seq::const_iterator s_iter;
		for (s_iter=seq.begin(); s_iter != seq.end(); s_iter++) {
			print_Chunk (*s_iter);
		}
		cout << "\t)\n} " << endl;

		
	}
}


bool check_nc_err(const int stat, const int line, const char *file) {
	if (stat != NC_NOERR) {
		cerr << "line" <<  line << "of" << file << ": " << nc_strerror(stat) << endl;
		return true;
	}
	return false;
}

///
///almost automatically generated netcdf function to store netcdf data files
///
bool LearningData::write_nc_data (string fileName, size_t numSeqs_len, int inputVectorSize, int targetVectorSize, FeatureVector& inputVector, FeatureVector& targetVector, vector<int>& seqLengthsVector, size_t& numTimesteps_len) {

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

}

///
///check limit parameters correspondence
///
bool LearningData::check_limits (FeaturesLimits params1, FeaturesLimits params2) {
	if (params1.minX != params2.minX)
		return false;
	if (params1.minY != params2.minY)
		return false;
	if (params1.minZ != params2.minZ)
		return false;
	if (params1.maxX != params2.maxX)
		return false;
	if (params1.maxY != params2.maxY)
		return false;
	if (params1.maxZ != params2.maxZ)
		return false;
	if (params1.minDuration != params2.minDuration)
		return false;
	if (params1.maxDuration != params2.maxDuration)
		return false;
	if (params1.minValLabel != params2.minValLabel)
		return false;
	if (params1.maxValLabel != params2.maxValLabel)
		return false;
	return true;
}


///
///Concatenate data sequences
///
bool LearningData::concatenate_datasets (string dir, string writeFileName) {
	boost::regex seqfile_re ("(.*)\\.seq");
	boost::cmatch matches;
	path p(dir);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		return false;
	}

	directory_iterator dir_iter(p), dir_end;
	DataSet data;
	FeaturesLimits featLimits;
	bool checkFlag = false;
	for(;dir_iter != dir_end; ++dir_iter) {
		string dirstring (dir_iter->leaf().c_str());
		char *dirchar = (char *)dirstring.c_str();

		if (boost::regex_match((const char*)dirchar, matches, seqfile_re)) {
			DataSet currentData;
			FeaturesLimits currentFeatLimits;
			string dataBaseName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << dataBaseName << endl;
			read_dataset (dataBaseName, currentData, currentFeatLimits);
			if (!checkFlag) {
				featLimits = currentFeatLimits;
				checkFlag = true;
			}
			else {
				assert (check_limits (featLimits, currentFeatLimits));
			}

			cout << "size current data: " << currentData.size() << endl;
			DataSet::iterator it = data.end();
			data.insert (it, currentData.begin(), currentData.end());
		}
	}
	cout << "size data: " << data.size() << endl;
	if (!write_dataset (writeFileName, data, featLimits) ) {
		cerr << "Error writing dataset file!" << endl;
		return false;
	}
	
	return true;


}

};  /* smlearning namespace */
