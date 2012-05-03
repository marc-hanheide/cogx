/** @file GNGSMRegion.cpp
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

#include <metalearning/GNGSMRegion.h>
#include <dynamic_system_modeling/dynamic_system_event_sequence_set.hh>
#include <cryssmex/cryssmex.hh>

namespace smlearning {

GNGSMRegion::GNGSMRegion (int idx, int smCtxtSize/*, int created = 0*/) :
	index(idx),
	sMContextSize (smCtxtSize)
{
	minValuesSMVector.resize(sMContextSize, -1.0);
	maxValuesSMVector.resize(sMContextSize, 1.0);
	// minValuesSMVector.resize(LearningData::motorVectorSize, -1.0);
	// maxValuesSMVector.resize(LearningData::motorVectorSize, 1.0);
		
		
		
}

GNGSMRegion::GNGSMRegion (GNGSMRegion& parentRegion, int idx, double cuttingValue, int cuttingIdx, LearningData::DataSet& inheritedData, bool firstRegion) :
	index (idx),
	minValuesSMVector (parentRegion.minValuesSMVector),
	maxValuesSMVector (parentRegion.maxValuesSMVector),
	sMContextSize (parentRegion.sMContextSize),
	data (inheritedData),
	inputqErrorsHistory (parentRegion.inputqErrorsHistory),
	outputqErrorsHistory (parentRegion.outputqErrorsHistory),
	inputqGraphSizeHistory (parentRegion.inputqGraphSizeHistory),
	outputqGraphSizeHistory (parentRegion.outputqGraphSizeHistory)
{ //,
	// startingPositionsHistory (parentRegion.startingPositionsHistory
	// ) {
		
	if (firstRegion){
		maxValuesSMVector[cuttingIdx] = cuttingValue;
	}else {
		minValuesSMVector[cuttingIdx] = cuttingValue;
	}
}


double GNGSMRegion::getAvgError () {

	return inputqErrorsHistory.back ();
}

void GNGSMRegion::updateErrorsHistory ()
{
	inputqErrorsHistory.push_back (static_cast<GNG_Quantizer*>(cryssmex.getInputQuantizer ())->getAvgError ());
	outputqErrorsHistory.push_back (static_cast<GNG_Quantizer*>(cryssmex.getOutputQuantizer ())->getAvgError ());
}

void GNGSMRegion::updateGraphSizeHistory ()
{
	inputqGraphSizeHistory.push_back (static_cast<GNG_Quantizer*>(cryssmex.getInputQuantizer ())->graphsize ());
	outputqGraphSizeHistory.push_back (static_cast<GNG_Quantizer*>(cryssmex.getOutputQuantizer ())->graphsize ());

}


void GNGSMRegion::saveMDLHistory ()
{
	stringstream ssindex;
	ssindex << index;
	static_cast<GNG_Quantizer*>(cryssmex.getInputQuantizer ())->saveMDLHistory ("mdlinput_region" + ssindex.str() + ".txt", true);
	static_cast<GNG_Quantizer*>(cryssmex.getOutputQuantizer ())->saveMDLHistory ("mdloutput_region"  + ssindex.str() + ".txt", true);
}

///
///save region limits and learner data into a file
///
bool GNGSMRegion::writeData (string fileName) {
	string regFileName = fileName + ".reg";
	ofstream writeFile (regFileName.c_str(), ios::out | ios::binary);
	if (!writeFile)
		return false;  

	writeFile.write ((const char*)&index, sizeof (index));
	write_vector<double> (writeFile, minValuesSMVector);
	write_vector<double> (writeFile, maxValuesSMVector);
	write_vector<double> (writeFile, inputqErrorsHistory);
	write_vector<double> (writeFile, outputqErrorsHistory);
	write_vector<double> (writeFile, inputqGraphSizeHistory);
	write_vector<double> (writeFile, outputqGraphSizeHistory);
	// write_vector<double> (writeFile, startingPositionsHistory);

	writeFile.close ();
	cryssmex.saveInputQuantizer (fileName + "_inputq.qnt");
	cryssmex.saveOutputQuantizer (fileName + "_outputq.qnt");
	return true;
}

///
///read region limits and learner data from a file
///
bool GNGSMRegion::readData (string fileName) {
	string regFileName = fileName + ".reg";
	ifstream readFile (regFileName.c_str(), ios::in | ios::binary);
	if (!readFile)
		return false;  

	readFile.read ((char *)&index, sizeof(index));
	read_vector<double> (readFile, minValuesSMVector);
	read_vector<double> (readFile, maxValuesSMVector);
	read_vector<double> (readFile, inputqErrorsHistory);
	read_vector<double> (readFile, outputqErrorsHistory);
	read_vector<double> (readFile, inputqGraphSizeHistory);
	read_vector<double> (readFile, outputqGraphSizeHistory);
	// read_vector<double> (readFile, startingPositionsHistory);
	
	readFile.close ();

	sMContextSize = minValuesSMVector.size ();

	cryssmex.setInputQuantizer (fileName + "_inputq.qnt");
	cryssmex.setOutputQuantizer (fileName + "_outputq.qnt");
	
	return true;
}

void GNGSMRegion::printData () {
	cout << "minValuesSMVector: " << endl;
	print_featvector<double> (minValuesSMVector);
	cout << endl << "maxValuesSMVector: " << endl;
	print_featvector<double> (maxValuesSMVector);
	cout << endl << "inputqerrorsHistory: " << endl;
	print_featvector<double> (inputqErrorsHistory);
	cout << endl << "outputqerrorsHistory: " << endl;
	print_featvector<double> (outputqErrorsHistory);
	cout << endl << "inputqgraphsizeHistory: " << endl;
	print_featvector<double> (inputqGraphSizeHistory);
	cout << endl << "outputqgraphsizeHistory: " << endl;
	print_featvector<double> (outputqGraphSizeHistory);
	// cout << endl << "startingPositionsHistory: " << endl;
	// print_featvector<double> (startingPositionsHistory);
	cout << endl;
}


bool GNGSMRegion::checkSMRegionMembership (const FeatureVector& sMContext)
{
	for (int i=0; i<sMContextSize; i++) {
		if (minValuesSMVector[i] > sMContext[i] || maxValuesSMVector[i] < sMContext[i])
			return false;
	}
	return true;
}
	
///
///Find the appropriate region index according to the given sensorimotor context
///
int GNGSMRegion::getSMRegion (const GNGSMRegion::RegionsMap& regions, const FeatureVector& sMContext) {
	for (RegionsMap::const_iterator regionIter = regions.begin(); regionIter != regions.end(); regionIter++) {
		bool wrongCuttingValue = false;
		// GNGSMRegion currentRegion = regionIter->second;
		//assert (sMContext.size() == currentRegion.sMContextSize);
		// cout << "sMContext size: " << sMContext.size() << endl;
		// cout << "motorVectorsize: " << SMRegion::motorVectorSize << endl;
		// assert (sMContext.size() == LearningData::motorVectorSize);
		for (int i=0; i<regionIter->second.sMContextSize; i++) {
		// for (int i=0; i<LearningData::motorVectorSize; i++) {
			if ( regionIter->second.minValuesSMVector[i] > sMContext[i] ||
			     regionIter->second.maxValuesSMVector[i] < sMContext[i]) {
				wrongCuttingValue = true;
				break;
			}
		}
		if (wrongCuttingValue) continue; else return regionIter->second.index;
	}
	return -1;
}

void GNGSMRegion::redirectOutputToNull ()
{
	static_cast<GNG_Quantizer*>(cryssmex.getInputQuantizer())->redirectOutput ("/dev/null");
	static_cast<GNG_Quantizer*>(cryssmex.getOutputQuantizer())->redirectOutput ("/dev/null");
}

void GNGSMRegion::generateCryssmexFiles (unsigned int max_iterations, ssm::SSM::Type ssm/*, bool save_all*/, string prefix, string regionFileName, LearningData::FeaturesLimits& limits, feature_selection featureSelectionMethod)
{
	LearningData::write_cryssmexdataset (regionFileName, data, normalize<double>, limits, featureSelectionMethod);

	// From here on, this is mostly copying CrySSMEx learning process
	dynamic_system_modeling::Dynamic_System_Event_Sequence_Set dsess(regionFileName + ".cry");
	cryssmex::CrySSMEx::Parameters cryssmex_parameters;
	cryssmex_parameters.ssm_type = ssm;
	cryssmex_parameters.merge_mode = cryssmex::CrySSMEx::undi;
	cryssmex_parameters.split_mode = cryssmex::CrySSMEx::basic;
	
	if (featureSelectionMethod == _obpose || featureSelectionMethod == _obpose_direction || featureSelectionMethod == _obpose_label || featureSelectionMethod == _obpose_rough_direction || featureSelectionMethod == _obpose_slide_flip_tilt || featureSelectionMethod == _mcobpose_obpose_direction) //suitable for Mealy machines
		cryssmex.initializeStateQuantizer (LearningData::pfVectorSize);

	else if (featureSelectionMethod == _efobpose || featureSelectionMethod == _efobpose_direction || featureSelectionMethod == _efobpose_label || featureSelectionMethod == _efobpose_rough_direction || featureSelectionMethod == _efobpose_slide_flip_tilt) //suitable for Moore machines
		cryssmex.initializeStateQuantizer (LearningData::efVectorSize + LearningData::pfVectorSize);

	cryssmex::CrySSMEx _cryssmex(dsess, cryssmex.getInputQuantizer(), cryssmex.getOutputQuantizer(), cryssmex.getStateQuantizer(), cryssmex_parameters);

	bool first_loop = true;
  
	string log_filename(prefix + regionFileName + "_cryssmex.log");
	cout << "Logging in " << log_filename << endl;

	ofstream log(log_filename.c_str());
	log << "# CrySSMEx log: " << log_filename << endl
	    << "# Col 1: iteration \n"
	    << "# Col 2: |Q|\n"
	    << "# Col 3: max level of CVQ graph\n"
	    << "# Col 4: node count of CVQ\n";

	unsigned int max_state_count = _cryssmex.ssm()->state_count();
	unsigned int iter_max_state_count = 0;
	while(!_cryssmex.ssm()->properties.test_determinism() && 
	      (_cryssmex.iteration() - iter_max_state_count < max_iterations)) {
		if (!first_loop) {
			_cryssmex.split();    
		}
    
		cout << "CrySSMEx iteration: " 
		     << _cryssmex.iteration() 
		     << " (" << _cryssmex.ssm()->state_count() 
		     << ")"<< endl;
    
		// if(save_all) {
		// 	save_cryssmex_stuff(_cryssmex, prefix + "cryssmex_", 1);
		// }
    
		log << _cryssmex.iteration()
		    << " " << _cryssmex.ssm()->state_count()
		    << " " << _cryssmex.cvq()->max_level()
		    << " " << _cryssmex.cvq()->node_count()
		    << endl;

		if (_cryssmex.ssm()->state_count() > max_state_count)
		{
			max_state_count = _cryssmex.ssm()->state_count();
			iter_max_state_count = _cryssmex.iteration();
		}
      
		first_loop = false;
		//save(cryssmex.ssm(), ssm_filename);
	}
	log.close();
  
	string ssm_filename(prefix + regionFileName + "_cryssmex_ssm_final.ssm");
	cout << "Saving " << ssm_filename << endl;
	save(_cryssmex.ssm(), ssm_filename);

	string cvq_filename(prefix + regionFileName + "_cryssmex_cvq_final.qnt");
	cout << "Saving " << cvq_filename << endl;
	save_quantizer(_cryssmex.cvq(), cvq_filename);

	
}



}; /* namespace smlearning */
