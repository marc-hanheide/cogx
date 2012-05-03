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

enum plot { errors, graphsize };

void scriptValues(string fileName, string sourceName, string scriptName, int region, plot plottype) {

	string descText1;
	string descText2;
	if (plottype == errors) {
		descText1 = "Errors history for Input Q.";
		descText2 = "Errors history for Output Q.";
	}
	else if (plottype == graphsize) {
		descText1 = "Graph size history for Input Q.";
		descText2 = "Graph size history for Output Q.";
	}
	

	stringstream gnuplotstream;

	gnuplotstream << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	gnuplotstream << "set output \"|epstopdf --filter > '" + fileName + ".pdf'" << endl;
	gnuplotstream << "set autoscale" << endl;
	gnuplotstream << "set style line 1 lt 1 lc rgb \"blue\"" << endl;
	gnuplotstream << "set style line 2 lt 1 lc rgb \"red\"" << endl;
	gnuplotstream << "set style line 3 lt 1 lc rgb \"green\"" << endl;
	gnuplotstream << "set style line 4 lt 1 lc rgb \"yellow\"" << endl;
	// gnuplotstream << "set log y" << endl;
	gnuplotstream << "set title \"Region " << region << endl;
	gnuplotstream << "set xlabel \"Iterations\" " << endl;
	gnuplotstream << "set ylabel \"Values\" " << endl;
	gnuplotstream << "plot  \"" + sourceName + "\" "; 
	gnuplotstream << "using 1:2 ls 1 title '" + descText1 +"' with lines, ";
	gnuplotstream << "\"" + sourceName + "\" "; 
	gnuplotstream << "using 1:3 ls 2 title '" + descText2 +"' with lines";
	gnuplotstream << endl;

	ofstream gnuplotScript(scriptName.c_str(), ios::out);
	gnuplotScript << gnuplotstream.str() << endl;
	gnuplotScript.close();

}

void plotAndFinish(string valuesFile, string scriptFile) {

	::system(("gnuplot " + scriptFile).c_str());
	
	::system(("rm " + valuesFile).c_str());
	::system(("rm " + scriptFile).c_str());
	
}

int main (int argc, char* argv[])
{

 	po::options_description desc("Allowed options");
	desc.add_options()
		("help,h", "produce help message")
		("regFile,r", po::value<string>(), "region file to plot")
		("firstIndex,f", po::value<int>(), "index to plot from")
		("lastIndex,l", po::value<int>(), "index to plot to")
	;
	
	po::variables_map vm;       
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);   
	
	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}


	if (!vm.count("regFile")) {
		cout << "No source files specified. Exciting..." << endl;
		cout << "Use  --help  for argument options."  << endl;
		return 1;
	}
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

	ofstream errorFileValues(tempValuesName.c_str(), ios::out);
	for (int i = firstIndex; i < lastIndex; i++) {
		errorFileValues << i << "\t";
		errorFileValues << region.inputqErrorsHistory[i] << "\t";
		errorFileValues << region.outputqErrorsHistory[i] << endl;
	}
	scriptValues((vm["regFile"].as<string>() + "-Errors"), tempValuesName, tempScriptName, region.index, errors);
	plotAndFinish(tempValuesName, tempScriptName);
	cout << endl << "region error history plot printed" << endl;

	ofstream graphsizeFileValues(tempValuesName.c_str(), ios::out);
	for (int i = firstIndex; i < lastIndex; i++) {
		graphsizeFileValues << i << "\t";
		graphsizeFileValues << region.inputqGraphSizeHistory[i] << "\t";
		graphsizeFileValues << region.outputqGraphSizeHistory[i] << endl;
	}
	scriptValues((vm["regFile"].as<string>() + "-GraphSize"), tempValuesName, tempScriptName, region.index, graphsize);
	plotAndFinish(tempValuesName, tempScriptName);
	cout << endl << "region graph size history plot printed" << endl;
	
}
