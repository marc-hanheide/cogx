/** @file evalplot_activelearning.cpp
 *
 * Generation of plots obtained by class \p ActiveLearnScenario
 *
 * @author      Sergio Roa - DFKI
 * @version 2.0 beta
 *
 */

#include <boost/program_options.hpp>
#include <metalearning/GNGSMRegion.h>

namespace po = boost::program_options;

using namespace smlearning;

typedef map<double, vector<unsigned int> > map_frequencies;

void scriptValues(string fileName, string sourceName, string scriptName, int region, vector<string> descText) {


	stringstream gnuplotstream;

	gnuplotstream << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	gnuplotstream << "set output \"|epstopdf --filter > '" + fileName + ".pdf'" << endl;
	gnuplotstream << "set autoscale" << endl;
	// gnuplotstream << "set log y" << endl;
	if (region != -1)
		gnuplotstream << "set title \"Region " << region << endl;
	gnuplotstream << "set xlabel \"Iterations\" " << endl;
	gnuplotstream << "set ylabel \"Values\" " << endl;
	gnuplotstream << "plot  ";
	for (int i=0; i<descText.size(); i++)
	{
		gnuplotstream << "\"" + sourceName + "\" "; 
		gnuplotstream << "using 1:" << i+2 << " lt " << i+1 << " lc " << i+1 << " title '" + descText[i] +"' with lines";
		if (i < descText.size() -1)
			gnuplotstream << ", ";
	}
	gnuplotstream << endl;

	ofstream gnuplotScript(scriptName.c_str(), ios::out);
	gnuplotScript << gnuplotstream.str() << endl;
	gnuplotScript.close();

}

map_frequencies getFrequencyStatistics (vector<double> startingPositionsHistory, int firstIndex, int lastIndex, int window)
{
	map_frequencies statistics;

	for (int i=firstIndex; i<lastIndex; i++)
	{
		if (startingPositionsHistory[i] != 0)
		{
			if (statistics.find (startingPositionsHistory[i]) == statistics.end ()) {
				vector<unsigned int> freqVector;
				statistics[startingPositionsHistory[i]] = freqVector;
			}
		}
	}
	
	for (int i=firstIndex; i<lastIndex; i++)
	{

		for(map_frequencies::const_iterator it = statistics.begin(); it != statistics.end(); ++it)
		{
			double k = it->first;
			statistics[k].push_back (0);
		}
		for (int j=i; j<i+window && j<lastIndex; j++)
		{
			if (startingPositionsHistory[j] != 0)
				statistics[startingPositionsHistory[j]].back()++;
		}
	}
	return statistics;
}

void plotAndFinish(string valuesFile, string scriptFile) {

	::system(("gnuplot " + scriptFile).c_str());
	
	::system(("rm " + valuesFile).c_str());
	::system(("rm " + scriptFile).c_str());
	
}

void generateFrequenciesPlot (string fileName, vector<double> startingPositionsHistory, int regionindex, int firstIndex, int lastIndex, string tempValuesName, string tempScriptName, unsigned int window)
{
	map_frequencies frequencystartpositions = getFrequencyStatistics (startingPositionsHistory, firstIndex, lastIndex, window);
	vector<string> descText;
	for (map_frequencies::const_iterator it = frequencystartpositions.begin(); it != frequencystartpositions.end(); ++it)
	{
		std::ostringstream s;
		s << (*it).first;
		if (it == frequencystartpositions.begin())
			descText.push_back ("Frequency of actions starting in position " + s.str ());
		else
			descText.push_back ("... in position " + s.str());
	}

	ofstream freqstartpositionsFileValues (tempValuesName.c_str(), ios::out);
	int freqVectorSize = (*frequencystartpositions.begin()).second.size();
	for (int i=0; i<freqVectorSize; i++)
	{
		freqstartpositionsFileValues << i << "\t";
		for (map_frequencies::const_iterator it = frequencystartpositions.begin(); it != frequencystartpositions.end(); ++it)
			freqstartpositionsFileValues << (*it).second[i] << "\t";
		freqstartpositionsFileValues << endl;
	}
	scriptValues (fileName + "-StartPosStats", tempValuesName, tempScriptName, regionindex, descText);
	plotAndFinish(tempValuesName, tempScriptName);

}

int main (int argc, char* argv[])
{

 	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "produce help message")
		("regFile,r", po::value<string>(), "region file to plot")
		("firstIndex,f", po::value<int>(), "index to plot from")
		("lastIndex,l", po::value<int>(), "index to plot to")
		("window,w", po::value<unsigned int>()->default_value(20), "window size for frequency of actions")
		("frequencysequences,d", po::value<string>(), "generate frequency of actions given a sequences data set file name (no extension)")
	;
	
	po::variables_map vm;       
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);   
	
	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}
	if (!vm.count("regFile") && !vm.count("frequencysequences")) {
		cout << "No source files specified. Use -r and/or -d options. Exiting..." << endl;
		cout << "Use  --help  for argument options."  << endl;
		return 1;
	}

	if (vm.count("regFile")) {
		GNGSMRegion region;
		int firstIndex = 0;

		if (!region.readData((vm["regFile"].as<string>()))) {
			cerr << "Specified reg file couldn't be read. Exiting..." << endl;
			return 1;
		} else {
			cout << endl << "reg file loaded." << endl;
		}

		int lastIndex = region.inputqErrorsHistory.size();

		if (vm.count("firstIndex")) {
			int fi = vm["firstIndex"].as<int>();
			if (fi < firstIndex || fi > lastIndex) {
				cerr << "Invalid first_index (out of range), using " << firstIndex << " instead." << endl;		
			} else {
				firstIndex = fi;
			}
		}

		if (vm.count("lastIndex")) {
			int li = vm["lastIndex"].as<int>();
			if (li < firstIndex || li > lastIndex) {
				cerr << "Invalid last_index (out of range), using " << lastIndex << " instead." << endl;		
			} else {
				lastIndex = li;
			}
		}

		cout << "Using " << firstIndex << " as first_index." << endl;
		cout << "Using " << lastIndex << " as last_index." << endl;

		string tempValuesName = "tempValues";	
		string tempScriptName = "tempgnuscript.gnu";

		vector<string> descText;
		descText.push_back ("Normalized model efficiency history for Input Q.");
		descText.push_back ("Normalized model efficiency history for Output Q.");
		ofstream errorFileValues(tempValuesName.c_str(), ios::out);
		for (int i = firstIndex; i < lastIndex; i++) {
			errorFileValues << i << "\t";
			errorFileValues << region.inputqErrorsHistory[i] << "\t";
			cout << region.inputqErrorsHistory[i] << "\t";
			errorFileValues << region.outputqErrorsHistory[i] << endl;
			cout << region.outputqErrorsHistory[i] << endl;
		}
		scriptValues((vm["regFile"].as<string>() + "-Errors"), tempValuesName, tempScriptName, region.index, descText);
		plotAndFinish(tempValuesName, tempScriptName);
		cout << endl << "region error history plot printed" << endl;

		descText.clear ();
		descText.push_back ("Graph size history for Input Q.");
		descText.push_back ("Graph size history for Output Q.");	
		ofstream graphsizeFileValues(tempValuesName.c_str(), ios::out);
		for (int i = firstIndex; i < lastIndex; i++) {
			graphsizeFileValues << i << "\t";
			graphsizeFileValues << region.inputqGraphSizeHistory[i] << "\t";
			graphsizeFileValues << region.outputqGraphSizeHistory[i] << endl;
		}
		scriptValues((vm["regFile"].as<string>() + "-GraphSize"), tempValuesName, tempScriptName, region.index, descText);
		plotAndFinish(tempValuesName, tempScriptName);
		cout << endl << "region graph size history plot printed" << endl;

		generateFrequenciesPlot (vm["regFile"].as<string>(), region.startingPositionsHistory, region.index, firstIndex, lastIndex, tempValuesName, tempScriptName, vm["window"].as<unsigned int>());
		cout << endl << "region frequency of actions history plot printed" << endl;
	}

	if (vm.count("frequencysequences")) {
		int firstIndex = 0;

		string stpFileName = vm["frequencysequences"].as<string>();
		ifstream readFile (string(stpFileName + ".stp").c_str(), ios::in | ios::binary);
		if (!readFile)
		{
			cout << "Sequences File could not be read..." << endl;
			return false;
		}
		vector<double> startingPositionsHistory;
		read_vector<double> (readFile, startingPositionsHistory);

		int lastIndex = startingPositionsHistory.size();

		if (vm.count("firstIndex")) {
			int fi = vm["firstIndex"].as<int>();
			if (fi < firstIndex || fi > lastIndex) {
				cerr << "Invalid first_index (out of range), using " << firstIndex << " instead." << endl;		
			} else {
				firstIndex = fi;
			}
		}

		if (vm.count("lastIndex")) {
			int li = vm["lastIndex"].as<int>();
			if (li < firstIndex || li > lastIndex) {
				cerr << "Invalid last_index (out of range), using " << lastIndex << " instead." << endl;		
			} else {
				lastIndex = li;
			}
		}

		cout << "Using " << firstIndex << " as first_index." << endl;
		cout << "Using " << lastIndex << " as last_index." << endl;

		string tempValuesName = "tempValues";	
		string tempScriptName = "tempgnuscript.gnu";


		generateFrequenciesPlot (stpFileName, startingPositionsHistory,-1, firstIndex, lastIndex, tempValuesName, tempScriptName, vm["window"].as<unsigned int>());
		cout << endl << "frequency of actions history plot printed" << endl;

	}
}
