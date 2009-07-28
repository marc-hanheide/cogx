/** @file data_handling.cpp
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2007,2008 Alex Graves
 *           2009      Sergio Roa

   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with nnl.  If not, see <http://www.gnu.org/licenses/>.
 
*/

#include <tools/data_handling.h>

namespace smlearning {

#define FEATUREVECTOR_SIZE1 8
#define FEATUREVECTOR_SIZE2 6

void generate_rand_sequences (DataSet& data, long numSeq, long seqSize) {
	// initialize random seed:
	srand ((unsigned)time(NULL) );
	
	// generate random number:
	double randNr;

	for (int s=0; s<numSeq; s++) {
		Sequence currentSequence;
		for (int v=0; v<seqSize; v++) {

			FeatureVector currentVector;
			int vectorSize;
			if (v==0)
				vectorSize = FEATUREVECTOR_SIZE1;
			else
				vectorSize = FEATUREVECTOR_SIZE2;
		
			for (int n=0; n< vectorSize; n++) {
				randNr = (rand() % 10 + 1) / 10.0;
				currentVector.push_back (randNr);			
			}

			
			currentSequence.push_back (currentVector);	
		}
		data.push_back (currentSequence);
	}

}

bool write_dataset (string fileName, const DataSet& data) {
	fileName += ".seq";
	ofstream writeFile(fileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;

	long numSeqs = data.size();
	writeFile.write ((const char*)&numSeqs, sizeof(numSeqs));
//  	cout << numSeqs << endl;
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		long seqSize = (*s).size();
		writeFile.write ((const char*)&seqSize, sizeof (seqSize));
//  		cout << "\t" << seqSize << endl;
		Sequence::const_iterator v;

		for (v=(*s).begin(); v!= (*s).end(); v++) {
			long featvectorSize = (*v).size();
			writeFile.write ((const char*)&featvectorSize, sizeof (featvectorSize));
//  			cout << "\t\t" << featvectorSize << endl;
			FeatureVector::const_iterator n;
			for (n=(*v).begin(); n!= (*v).end(); n++) {
				writeFile.write ((const char* )&(*n), sizeof (*n));
			}
		}
	}
	
	writeFile.close();
	return true;
}

bool read_dataset (string fileName, DataSet& data) {
	fileName += ".seq";
	ifstream readFile(fileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;

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
			long featvectorSize;
			readFile.read ((char *)&featvectorSize, sizeof(featvectorSize));
// 			cout << "\t\t" << featvectorSize << endl;
			for (int n=0; n<featvectorSize; n++) {
				double value;
				readFile.read ((char* )&value, sizeof(value));
				currentVector.push_back (value);
			}
			currentSequence.push_back (currentVector);
		}
		data.push_back(currentSequence);
	}	
	
	readFile.close();
	return true;



	
}

//write a cdl file format with zero padding
bool write_cdl_file_padding (string fileName, const DataSet& data) {
	fileName += ".cdl";
	ofstream writeFile(fileName.c_str(), ios::out);
	if (!writeFile)
		return false;

	long maxfeatvectorSize = -1;

	//find max. feature vector size which will be inputPattSize and targetPattSize
	Sequence::const_iterator v = (*(data.begin())).begin();
	for (int i=0; i<2; i++,v++) {
		long featvectorSize = (*v).size();
		if (featvectorSize > maxfeatvectorSize)
			maxfeatvectorSize = featvectorSize;
	}
		
	stringstream seqLengthsStr;
	stringstream inputsStr;
	stringstream targetPatternsStr;
	long numTimesteps = 0;
	seqLengthsStr << "\tseqLengths = ";
	inputsStr << "\tinputs =" << endl << "\t\t";
	targetPatternsStr << "\ttargetPatterns = " << endl << "\t\t";
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		long seqSize = (*s).size() - 1;
		seqLengthsStr << seqSize;
		numTimesteps += seqSize;
		if (s+1 == data.end())
			seqLengthsStr << ";";
		else
			seqLengthsStr << ", ";

		Sequence::const_iterator v;
		for (v=(*s).begin(); v!= (*s).end(); v++) {
			long featvectorSize = (*v).size();
			long paddingSize = maxfeatvectorSize - featvectorSize;
			FeatureVector::const_iterator n;
			//put inputs and targetPatterns data
			if (v+1 != (*s).end()) {
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					if (n != (*v).begin())
						inputsStr << ", ";
					inputsStr << *n;
				}
				//zero padding
				for (int i=0; i<paddingSize; i++)
					inputsStr << ", 0";
				if (v+2 == (*s).end() && s+1 == data.end())
					inputsStr << ";";
				else
					inputsStr << "," << endl << "\t\t";
			}
			if (v != (*s).begin()) {
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					if (n != (*v).begin())
						targetPatternsStr << ", ";
					targetPatternsStr << *n;
				}
				//zero padding
				for (int i=0; i<paddingSize; i++)
					targetPatternsStr << ", 0";
				if (v+1 == (*s).end() && s+1 == data.end())
					targetPatternsStr << ";";
				else
					targetPatternsStr << "," << endl << "\t\t";
			}

			
		}
	}

	writeFile << "netcdf " << fileName << " {" << endl;
	int numSeqs = data.size();
	writeFile << "dimensions:" << endl;
	writeFile << "\tnumSeqs = " << numSeqs << ",\n";
	writeFile << "\tnumTimesteps = " << numTimesteps << "," << endl;
	writeFile << "\tinputPattSize = " << maxfeatvectorSize << "," << endl;
	writeFile << "\ttargetPattSize = " << maxfeatvectorSize << ";" << endl;
	writeFile << "variables:" << endl << "\tfloat inputs(numTimesteps, inputPattSize);" \
		  << endl << "\tint seqLengths(numSeqs);" << endl \
		  << "\tfloat targetPatterns(numTimesteps, targetPattSize);" << endl;
	writeFile << "data:" << endl;
	writeFile << inputsStr.str() << endl;
	writeFile << seqLengthsStr.str() << endl;
	writeFile << targetPatternsStr.str() << endl;
	writeFile << "}" << endl;

	writeFile.close ();
	return true;
}

//write a cdl file format for feature vectors using basis vectors
bool write_cdl_file_basis (string fileName, const DataSet& data) {
	fileName += ".cdl";
	ofstream writeFile(fileName.c_str(), ios::out);
	if (!writeFile)
		return false;

	int initialVectorSize = data[0][0].size();
	int featureVectorSize = initialVectorSize + data[0][1].size();

	stringstream seqLengthsStr;
	stringstream inputsStr;
	stringstream targetPatternsStr;
	long numTimesteps = 0;
	seqLengthsStr << "\tseqLengths = ";
	inputsStr << "\tinputs =" << endl << "\t\t";
	targetPatternsStr << "\ttargetPatterns = " << endl << "\t\t";
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		long seqSize = (*s).size() - 1;
		seqLengthsStr << seqSize;
		numTimesteps += seqSize;
		if (s+1 == data.end())
			seqLengthsStr << ";";
		else
			seqLengthsStr << ", ";

		Sequence::const_iterator v;
		for (v=(*s).begin(); v!= (*s).end(); v++) {
			FeatureVector::const_iterator n;
			//put inputs and targetPatterns data
			if (v+1 != (*s).end()) {
				if (v != (*s).begin()) {
					//zero padding
					for (int i=0; i<initialVectorSize; i++)
						inputsStr << "0, ";
				}
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					if (n != (*v).begin())
						inputsStr << ", ";
					inputsStr << *n;
				}
				if (v == (*s).begin()) {
					//zero padding
					for (int i=0; i<featureVectorSize - initialVectorSize; i++)
						inputsStr << ", 0";
				}
				if (v+2 == (*s).end() && s+1 == data.end())
					inputsStr << ";";
				else
					inputsStr << "," << endl << "\t\t";
			}
			
			if (v != (*s).begin()) {
				if (v != (*s).begin()) {
					//zero padding
					for (int i=0; i<initialVectorSize; i++)
						targetPatternsStr << "0, ";
				}
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					if (n != (*v).begin())
						targetPatternsStr << ", ";
					targetPatternsStr << *n;
				}
				//zero padding
				if (v == (*s).begin() ) {
					for (int i=0; i<featureVectorSize - initialVectorSize; i++)
						targetPatternsStr << ", 0";
				}
				if (v+1 == (*s).end() && s+1 == data.end())
					targetPatternsStr << ";";
				else
					targetPatternsStr << "," << endl << "\t\t";
			}
		}
	}

	writeFile << "netcdf " << fileName << " {" << endl;
	int numSeqs = data.size();
	writeFile << "dimensions:" << endl;
	writeFile << "\tnumSeqs = " << numSeqs << ",\n";
	writeFile << "\tnumTimesteps = " << numTimesteps << "," << endl;
	writeFile << "\tinputPattSize = " << featureVectorSize << "," << endl;
	writeFile << "\ttargetPattSize = " << featureVectorSize << ";" << endl;
	writeFile << "variables:" << endl << "\tfloat inputs(numTimesteps, inputPattSize);" \
		  << endl << "\tint seqLengths(numSeqs);" << endl \
		  << "\tfloat targetPatterns(numTimesteps, targetPattSize);" << endl;
	writeFile << "data:" << endl;
	writeFile << inputsStr.str() << endl;
	writeFile << seqLengthsStr.str() << endl;
	writeFile << targetPatternsStr.str() << endl;
	writeFile << "}" << endl;

	writeFile.close ();
	return true;
	
}

bool check_nc_err(const int stat, const int line, const char *file) {
	if (stat != NC_NOERR) {
		cerr << "line" <<  line << "of" << file << ": " << nc_strerror(stat) << endl;
		return true;
	}
	return false;
}


bool write_nc_data (string fileName, const DataSet& data, int featureVectorSize, FeatureVector& inputVector, FeatureVector& targetVector, vector<int>& seqLengthsVector, size_t& numTimesteps_len) {

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
	size_t inputPattSize_len = featureVectorSize;
	size_t targetPattSize_len = featureVectorSize;
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

	cout << "numTimesteps: " << numTimesteps_len << endl << "inputPattSize: " << inputPattSize_len << endl;
	cout << "inputVector.size(): " << inputVector.size() << endl;
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


//write a netcdf nc file format for feature vectors using basis vectors
bool write_nc_file_basis (string fileName, const DataSet& data) {
	fileName += ".nc";

	int initialVectorSize = data[0][0].size();
	int featureVectorSize = initialVectorSize + data[0][1].size();

	FeatureVector inputVector;
	FeatureVector targetVector;
	vector<int> seqLengthsVector;
	size_t numTimesteps_len = 0;
	DataSet::const_iterator s;
	for (s=data.begin(); s!= data.end(); s++) {
		size_t seqSize = (*s).size() - 1;
		seqLengthsVector.push_back( seqSize );
		numTimesteps_len += seqSize;

		Sequence::const_iterator v;
		for (v=(*s).begin(); v!= (*s).end(); v++) {
			FeatureVector::const_iterator n;
			//put inputs and targetPatterns data
			if (v+1 != (*s).end()) {
				if (v != (*s).begin()) {
					//zero padding
					for (int i=0; i<initialVectorSize; i++)
						inputVector.push_back (0.0);
				}
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					inputVector.push_back (*n);
				}
				if (v == (*s).begin()) {
					//zero padding
					for (int i=0; i<featureVectorSize - initialVectorSize; i++)
						inputVector.push_back (0.0);
				}
			}
			
			if (v != (*s).begin()) {
				if (v != (*s).begin()) {
					//zero padding
					for (int i=0; i<initialVectorSize; i++)
						targetVector.push_back (0.0);
				}
				for (n=(*v).begin(); n!= (*v).end(); n++) {
					targetVector.push_back (*n);
				}
				//zero padding
				if (v == (*s).begin() ) {
					for (int i=0; i<featureVectorSize - initialVectorSize; i++)
						targetVector.push_back (0.0);
				}
			}
		}
	}

	//netcdf file storing
	if (!write_nc_data (fileName, data, featureVectorSize, inputVector, targetVector, seqLengthsVector, numTimesteps_len))
		return false;

	return true;
	
}


bool concatenate_datasets (string dir, string writeFileName) {
	boost::regex seqfile_re ("(.*)\\.seq");
	boost::cmatch matches;
	path p(dir);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		return false;
	}

	directory_iterator dir_iter(p), dir_end;
	DataSet data;
	for(;dir_iter != dir_end; ++dir_iter) {
		if (boost::regex_match(dir_iter->leaf().c_str(), matches, seqfile_re)) {
			DataSet currentData;
			string dataBaseName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << dataBaseName << endl;
			read_dataset (dataBaseName, currentData);
			cout << "size current data: " << currentData.size() << endl;
			DataSet::iterator it = data.end();
			data.insert (it, currentData.begin(), currentData.end());
		}
	}
	cout << "size data: " << data.size() << endl;
	if (!write_dataset (writeFileName, data) ) {
		cerr << "Error writing dataset file!" << endl;
		return false;
	}
	
	return true;
}
	
void saveElement(ostream& out, const string& name, bool val, bool xml)
{
	if (xml)
	{
		out << "\t\t<" << name << ">" << val << "</" << name << ">" << endl;
	}
	else
	{
		out << name << " " << val << endl;
	}
}

void saveElement(ostream& out, const string& name, int val, bool xml)
{
	if (xml)
	{
		out << "\t\t<" << name << ">" << val << "</" << name << ">" << endl;
	}
	else
	{
		out << name << " " << val << endl;
	}
}

void saveElement(ostream& out, const string& name, const string&val, bool xml)
{
	if (xml)
	{
		out << "\t\t<" << name << ">" << val << "</" << name << ">" << endl;
	}
	else
	{
		out << name << " " << val << endl;
	}
}

void saveElement(ostream& out, const string& name, const vector<string>&val, bool xml)
{
	if (xml)
	{
		out << "\t\t<" << name << ">";
		copy(val.begin(), val.end(), ostream_iterator<string>(out, " "));
		out << "</" << name << ">" << endl;
	}
	else
	{
		out << name << " ";
		copy(val.begin(), val.end(), ostream_iterator<string>(out, " "));
		out << endl;
	}
}

void saveElement(ostream& out, const string& name, double val, bool xml)
{
	if (xml)
	{
		out << "\t\t<" << name << ">" << val << "</" << name << ">" << endl;
	}
	else
	{
		out << name << " " << val << endl;
	}
}

void OfflineRNN::saveConsoleData(ostream& out, bool xml)
{
	saveElement (out, "trainDataFile", trainDataFiles, xml);
	saveElement (out, "trainDataFraction", trainDataFraction, xml);
	saveElement (out, "testDataFraction", testDataFraction, xml);
	saveElement (out, "valDataFraction", valDataFraction, xml);
	saveElement (out, "testDataFile", testDataFile, xml);
	saveElement (out, "validationDataFile", valDataFile, xml);
	saveElement (out, "randomSeed", randomSeed, xml);
	saveElement (out, "totalEpochs", totalEpochs, xml);
	saveElement (out, "initWeightRange", initWeightRange, xml);
	saveElement (out, "initErrorTest", initErrorTest, xml);
	saveElement (out, "dataCheck", dataCheck, xml);
	saveElement (out, "gradTest", gradTest, xml);
	saveElement (out, "gradTestPerturbation", gradTestPerturbation, xml);
	saveElement (out, "gradTestSeq", gradTestSeq, xml);
	saveElement (out, "overwriteSaves", overwriteSaves, xml);
	saveElement (out, "staticNoise", staticNoise, xml );
	saveElement (out, "epochsPerErrCheck", epochsPerErrCheck, xml);
	saveElement (out, "epochsSinceErrCheck", epochsSinceErrCheck, xml);
	saveElement (out, "maxTestsNoBest", maxTestsNoBest, xml);
	saveElement (out, "epoch", epoch, xml);
	saveElement (out, "batchLearn", batchLearn, xml);
	saveElement (out, "shuffleTrainData", shuffleTrainData, xml);
	saveElement (out, "saveAfterNSeqs", saveAfterNSeqs, xml);
	saveElement (out, "seqsPerWeightUpdate", seqsPerWeightUpdate, xml);
	saveElement (out, "sequenceDebugOutput", SequenceDebugOutput::instance().get(), xml);
	saveElement (out, "rProp", rProp, xml);
	saveElement (out, "testOutputsFile", testOutputsFile, xml);
}

void OfflineRNN::save_net (ostream& out)
{
	out << "<NeuralNet>" << endl;
	out << "\t<ConsoleData>" << endl;
	saveConsoleData(out);
	out << "\t</ConsoleData>" << endl;
	net->save("\t", out);
	DataExportHandler::instance().save("\t", out);
	out << "</NeuralNet>" << endl;
}
	

void OfflineRNN::print_net_data (ostream& out)
{
	out << "console data:" << endl;
	saveConsoleData(out, false);
	out << endl;
	out << "network:" << endl;
	net->print(out);
	out << "gradientFollower:" << endl;
	gradientFollower->print(out);
	out << endl;
	if (trainData)
	{
		out << "training data:" << endl;
		trainData->print(out);
		out << endl;
	}
	if (testData)
	{
		out << "test data:" << endl;
		testData->print(out);
		out << endl;
	}
	if (valData)
	{
		out << "validation data:" << endl;
		valData->print(out);
		out << endl;
	}
}

NetcdfDataset* getValidDataset(NetcdfDataset* trainData, NetcdfDataset* testData, NetcdfDataset* valData)
{
	if (trainData)
	{
		return trainData;
	}
	else if (testData)
	{
		return testData;
	}
	else
	{
		return valData;
	}
}

const char* getDOMNodeChildText(const DOMElement* parent, const char* childName)
{
	const DOMNode* node = parent->getElementsByTagName(XMLString::transcode(childName))->item(0);
	if (node && node->hasChildNodes())
	{
		return XMLString::transcode(node->getFirstChild()->getNodeValue());
	}
	return 0;
}


bool loadDOMElement(const DOMElement* parent, const char* name, string& target)
{
	const char* text = getDOMNodeChildText(parent, name);
	if (text)
	{
		target = text;
		return true;
	}
	return false;
}

bool loadDOMElement(const DOMElement* parent, const char* name, vector<string>& target)
{
	const char* text = getDOMNodeChildText(parent, name);
	if (text)
	{
		stringstream temp(text);
		string s;
		while (temp >> s)
		{
			target.push_back(s);
		}
		return true;
	}
	return false;
}

bool loadDOMElement(const DOMElement* parent, const char* name, int& target)
{
	const char* text = getDOMNodeChildText(parent, name);
	if (text)
	{
		target = atoi(text);
		return true;
	}
	return false;
}

bool loadDOMElement(const DOMElement* parent, const char* name, double& target)
{
	const char* text = getDOMNodeChildText(parent, name);
	if (text)
	{
		target = atof(text);
		return true;
	}
	return false;
}

bool loadDOMElement(const DOMElement* parent, const char* name, bool& target)
{
	const char* text = getDOMNodeChildText(parent, name);
	if (text)
	{
		target = (atoi(text) != 0);
		return true;
	}
	return false;
}

//TODO fix xml validation
bool OfflineRNN::parse_netfile (XercesDOMParser& parser, const string& filename, bool display) {
	//parse in the file
	cout << "opening file \"" << filename << "\"" << endl;
	bool parseFailed = false;
	//XercesDOMParser parser;
	//parser.setValidationScheme(XercesDOMParser::Val_Always);
	parser.setDoNamespaces(true);
	//parser.setDoSchema(true);
	//parser.setValidationSchemaFullChecking(true); 
	//parser.setExternalNoNamespaceSchemaLocation("schema.xsd");
	parser.setValidationConstraintFatal(true);
	parser.setExitOnFirstFatalError(false);
	parser.setIncludeIgnorableWhitespace(false);
	try
	{
		ifstream f(filename.c_str());
		if (f.is_open())
		{
			LocalFileInputSource source(XMLString::transcode(filename.c_str()));
			parser.parse(source);
			//parseFailed = parser.getErrorCount() != 0;
			//Grammar* g = parser.getRootGrammar();
			if (parser.getErrorCount())
			{
				cerr << "Parsing " << filename;
				cerr << " error count: " << parser.getErrorCount() << endl;
				return false;
			}
		}
		else
		{
			cerr << "save file " << filename << " not found, exiting" << endl;
			return false;
		}
	}
	catch (const DOMException& e)
	{
		cerr << "DOM Exception parsing ";
		cerr << filename;
		cerr << " reports: ";
		// was message provided?
		if (e.msg)
		{
			// yes: display it as ascii.
			char *strMsg = XMLString::transcode(e.msg);
			cerr << strMsg << endl;
			XMLString::release(&strMsg);
		}
		else
		{
			// no: just display the error code.
			cerr << e.code << endl;
		}
		parseFailed = true;
	}
	catch (const XMLException& e)
	{
		cerr << "XML Exception parsing ";
		cerr << filename;
		cerr << " reports: ";
		cerr << e.getMessage() << endl;
		parseFailed = true;
	}
	catch (const SAXException& e)
	{
		cerr << "SAX Exception parsing ";
		cerr << filename;
		cerr << " reports: ";
		cerr << e.getMessage() << endl;
		parseFailed = true;
	}
	catch (...)
	{
		cerr << "An exception parsing ";
		cerr << filename << endl;
		parseFailed = true;
	}

	// did the input document parse okay?
	if (!parseFailed)
	{
		//TODOMNode* domNode = attributes->getNamedItem\(XMLString\:\:transcodeDO search for top nodes by name instead of number (order shouldn't matter)
		rootNode  = parser.getDocument()->getDocumentElement();
		consoleDataElement = (DOMElement*)rootNode->getElementsByTagName(XMLString::transcode("ConsoleData"))->item(0);		
		if (consoleDataElement)
		{
			bool sequenceDebugOutput = display;
			loadDOMElement(consoleDataElement, "trainDataFile", trainDataFiles);
			loadDOMElement(consoleDataElement, "testDataFile", testDataFile);			
			loadDOMElement(consoleDataElement, "validationDataFile", valDataFile);
			loadDOMElement(consoleDataElement, "randomSeed", randomSeed);
			loadDOMElement(consoleDataElement, "totalEpochs", totalEpochs);
			loadDOMElement(consoleDataElement, "initWeightRange", initWeightRange);
			loadDOMElement(consoleDataElement, "initErrorTest", initErrorTest);
			loadDOMElement(consoleDataElement, "trainDataFraction", trainDataFraction);
			loadDOMElement(consoleDataElement, "testDataFraction", testDataFraction);
			loadDOMElement(consoleDataElement, "valDataFraction", valDataFraction);
			loadDOMElement(consoleDataElement, "gradTest", gradTest);
			loadDOMElement(consoleDataElement, "gradTestPerturbation", gradTestPerturbation);
			loadDOMElement(consoleDataElement, "gradTestSeq", gradTestSeq);
			loadDOMElement(consoleDataElement, "epochsPerErrCheck", epochsPerErrCheck);
			loadDOMElement(consoleDataElement, "epochsSinceErrCheck", epochsSinceErrCheck);	
			loadDOMElement(consoleDataElement, "maxTestsNoBest", maxTestsNoBest);
			loadDOMElement(consoleDataElement, "epoch", epoch);
			loadDOMElement(consoleDataElement, "overwriteSaves", overwriteSaves);
			loadDOMElement(consoleDataElement, "staticNoise", staticNoise);
			loadDOMElement(consoleDataElement, "batchLearn", batchLearn);
			loadDOMElement(consoleDataElement, "shuffleTrainData", shuffleTrainData);
			loadDOMElement(consoleDataElement, "saveAfterNSeqs", saveAfterNSeqs);
			loadDOMElement(consoleDataElement, "seqsPerWeightUpdate", seqsPerWeightUpdate);
			loadDOMElement(consoleDataElement, "sequenceDebugOutput", sequenceDebugOutput);
			loadDOMElement(consoleDataElement, "rProp", rProp);
			loadDOMElement(consoleDataElement, "testOutputsFile", testOutputsFile);
			loadDOMElement(consoleDataElement, "dataCheck", dataCheck);
			SequenceDebugOutput::instance().set (sequenceDebugOutput);
		}
		else
		{
			cerr << "ConsoleData element not found!" << endl;
			return false;
		}

		netElement = (DOMElement*)rootNode->getElementsByTagName(XMLString::transcode("Net"))->item(0);
		if (!netElement)
		{
			cerr << "Net element not found!" << endl;
			return false;
		}
		//load in the exported data
		exportDataElement = (DOMElement*)rootNode->getElementsByTagName(XMLString::transcode("ExportedData"))->item(0);
		return true;
	}
	cerr << "xml parse failed!" << endl;
	return false;
	
}

	
bool OfflineRNN::load_net (int displaySeq, const string& displayDataSet)
{
	if (consoleDataElement)
	{
		//load in the datasets
		if (trainDataFiles.size() && trainDataFraction > 0 && (displaySeq < 0 || displayDataSet == "train"))
		{
			cout << "loading train data file \"" << trainDataFiles.front() << "\"" << endl;
			int seqOffset;
			if (displaySeq >= 0)
			{
				seqOffset = displaySeq;
				trainData = new NetcdfDataset(trainDataFiles.front(), seqOffset);
			}
			else
			{
				seqOffset = 0;
				trainData = new NetcdfDataset(trainDataFiles.front(), trainDataFraction);
			}
		}
		if (testDataFile != "" && testDataFraction > 0 && (displaySeq < 0 || displayDataSet == "test"))
		{
			cout << "loading test data file \"" << testDataFile << "\"" << endl;
			int seqOffset;
			if (displaySeq >= 0)
			{
				seqOffset = displaySeq;
				testData = new NetcdfDataset(testDataFile, seqOffset);
			}
			else
			{
				seqOffset = 0;
				testData = new NetcdfDataset(testDataFile, testDataFraction);
			}
		}
		if (valDataFile != "" && valDataFraction > 0 && (displaySeq < 0 || displayDataSet == "val"))
		{
			cout << "loading validation data file \"" << valDataFile << "\"" << endl;
			int seqOffset;
			if (displaySeq >= 0)
			{
				seqOffset = displaySeq;
				valData = new NetcdfDataset(valDataFile, seqOffset);
			}
			else
			{
				seqOffset = 0;
				valData = new NetcdfDataset(valDataFile, valDataFraction);
			}
		}

		NetcdfDataset* ds = getValidDataset(trainData, testData, valData);

		//if datasets loaded succesfully, load net
		if (ds)
		{
			if (netElement)
			{
				cout << "loading net" << endl;	

				//create a new net
				criteria.clear();
				//delete net;
				net = NetMaker::makeNet(netElement, ds->getDimensions(), criteria);
				for (VSCI it = criteria.begin(); it != criteria.end(); ++it)
				{
					bestTestErrors[*it] = bestValErrors[*it] = make_pair(numeric_limits<double>::max(), -1);
				}
					
				//build the net and create the gradientFollower
				net->build();
				if (rProp)
				{
					gradientFollower = GradientFollowerMaker::makeRpropGradientFollower(!batchLearn);
				}
				else
				{
					gradientFollower = GradientFollowerMaker::makeStdGradientFollower();
				}

			}
			else
			{
				cerr << "Net element not found, exiting" << endl;
				return false;
			}

			//load in the exported data
			if (exportDataElement)
			{
				cout << "loading exported data" << endl;
				DataExportHandler::instance().load(exportDataElement);
			}
			return true;
		}
		else
		{
			cerr << "no data loaded!" << endl;
			return false;
		}
	}
	else
	{
		cerr << "ConsoleData element not found!" << endl;
		return false;
	}
	cerr << "net loading failed!" << endl;
	return false;
}

void OfflineRNN::setTestDataFile (string fileName) {
	testDataFile = fileName;
}

void OfflineRNN::setTrainDataFile (string fileName) {
	trainDataFiles.clear();
	trainDataFiles.push_back (fileName);
}


bool generate_network_files_nfoldcv_set (const string defaultnetFileName, const string baseDataFileName, int n, string target_dir ) {
	//init the xerces lib
	try 
	{
		XMLPlatformUtils::Initialize();
	}
	catch (const XMLException& toCatch) 
	{
		char* message = XMLString::transcode(toCatch.getMessage());
		cerr << "Error during xerces initialization! :" << endl << message << endl;
		XMLString::release(&message);
	}

	OfflineRNN myRNN;
	XercesDOMParser parser;
	if (!myRNN.parse_netfile (parser, defaultnetFileName)) {
		cerr << "XML net file parsing not successful..." << endl;
		return false;
	}
	for (int i=0; i<n; i++) {
		stringstream testingFileName;
		stringstream trainingFileName;
		stringstream netFileName;
		netFileName << target_dir << "/" << baseDataFileName << "_" << n << "_foldcv_set-" << i << ".xml";
		ofstream netFile (netFileName.str().c_str());
		testingFileName << baseDataFileName << "_" << n << "_foldcv_set-" << i << "_testing.nc";
		trainingFileName << baseDataFileName << "_" << n << "_foldcv_set-" << i << "_training.nc";
		myRNN.setTestDataFile (testingFileName.str());
		myRNN.setTrainDataFile (trainingFileName.str());
		myRNN.load_net (1, "train");
		myRNN.print_net_data ();
		myRNN.save_net (netFile);
		netFile.close();
	}

	return true;
}
	
}; /* smlearning namespace */
