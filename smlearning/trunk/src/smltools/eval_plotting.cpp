#include <metalearning/Scenario.h>
//#include<tools/helpers.h>
#include <metalearning/SMRegion.h>

#include <boost/program_options.hpp>




namespace plotting {

void orderStartPosHistoryValues(int dis, vector<vector<int> >* statistics, int windowSize, vector<double>* values, int group, int zero, int firstIndex, int lastIndex ) {


	vector<int> startingPositions(dis, 0);
	((*statistics)).push_back(startingPositions);
	for (int i = 0; i < windowSize; i++) {
		if ((*values)[i] != 0) {
			startingPositions[((*values)[i+firstIndex]/group)]++;
		} else {
			startingPositions[zero]++;
		}
	}
	(*statistics).push_back(startingPositions);	

	for (int j = windowSize; j < lastIndex/*(*values).size()*/; j++){
		vector<int> newStat((*statistics).back());
		if ((*values)[(j-windowSize+firstIndex)] != 0) {
			newStat[((*values)[(j-windowSize+firstIndex)]/group)]--;	
		} else {
			newStat[zero]--;
		}
		if ((*values)[j+firstIndex] != 0) {
			newStat[((*values)[j+firstIndex]/group)]++;
		}  else {
			newStat[zero]++;
		}
		(*statistics).push_back(newStat);
	}

}



void printStartPosHistoryValues(string name, vector<vector<int> >* statistics) {
	
	ofstream tempStartPosFileValues(name.c_str(), ios::out);
	for (int i = 1; i < (*statistics).size(); i++) {
		tempStartPosFileValues << (i-1) << "\t";
		for (int j = 0; j < (*statistics)[0].size(); j++) {	
			tempStartPosFileValues << (*statistics)[i][j] << "\t";
		}
		tempStartPosFileValues << endl;
	}

}


void scriptValues(string fileName, string sourceName, string scriptName, int windowSize, int region=-1, bool learnProgErr = false) {

	string descText1;
	string descText2;
	if (learnProgErr) {
		descText1 = "Learning progress history";
		descText2 = "Errors history";
	} else {
		descText1 = "starting positions 1 - 9";
		descText2 = "starting positions 10 - 18";
	}
	

	stringstream actionsForStartPosGnuplotScript;

	actionsForStartPosGnuplotScript << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	actionsForStartPosGnuplotScript << "set output \"|epstopdf --filter > '" + fileName + ".pdf'" << endl;
	actionsForStartPosGnuplotScript << "set autoscale" << endl;
	actionsForStartPosGnuplotScript << "set style line 1 lt 1 lc rgb \"blue\"" << endl;
	actionsForStartPosGnuplotScript << "set style line 2 lt 1 lc rgb \"red\"" << endl;
	actionsForStartPosGnuplotScript << "set style line 3 lt 1 lc rgb \"green\"" << endl;
	actionsForStartPosGnuplotScript << "set style line 4 lt 1 lc rgb \"yellow\"" << endl;
	// actionsForStartPosGnuplotScript << "set log y" << endl;
	if (region != -1) {
		actionsForStartPosGnuplotScript << "set title \"Region " << region /*<< " (window size: " << windowSize << ")\""*/ << endl;
	} else {
		actionsForStartPosGnuplotScript << "set title \" Frequency of actions (window size: " << windowSize << ")\"" << endl;
	}
	actionsForStartPosGnuplotScript << "set xlabel \"Iterations\" " << endl;
	actionsForStartPosGnuplotScript << "set ylabel \"Values\" " << endl;
	actionsForStartPosGnuplotScript << "plot  \"" + sourceName + "\" "; 
	actionsForStartPosGnuplotScript << "using 1:2 ls 1 title '" + descText1 +"' with lines, ";
	actionsForStartPosGnuplotScript << "\"" + sourceName + "\" "; 
	actionsForStartPosGnuplotScript << "using 1:3 ls 2 title '" + descText2 +"' with lines";
	if (learnProgErr) {
		actionsForStartPosGnuplotScript << endl;
	} else {
		// actionsForStartPosGnuplotScript << ", \"" + sourceName + "\" "; 
		// actionsForStartPosGnuplotScript << "using 1:4 ls 3 title 'starting positions 19 - 24' with lines";
		if (region != -1) {
			actionsForStartPosGnuplotScript << ", \"" + sourceName + "\" "; 
			actionsForStartPosGnuplotScript << "using 1:5 ls 4 title 'current region not active' with lines" << endl;
		} else {
			actionsForStartPosGnuplotScript << endl;
		}
	}
	ofstream StartPosgnuplotScript(scriptName.c_str(), ios::out);
	StartPosgnuplotScript << actionsForStartPosGnuplotScript.str() << endl;
	StartPosgnuplotScript.close();

}





void plotAndFinish(string valuesFile, string scriptFile) {
	
	::system(("gnuplot " + scriptFile).c_str());
	
	::system(("rm " + valuesFile).c_str());
	::system(("rm " + scriptFile).c_str());
	
}


}



namespace po = boost::program_options;

using namespace smlearning;
using namespace plotting;

	
int main(int ac, char* av[]) {

try {

SMRegion region;
vector<double> experimentStartingPositions;
int windowSize = 20;
int firstIndex = 0;
int lastIndex;



 	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")
		("regFile,R", po::value<string>(), "region file to plot")
		("stpFile,S", po::value<string>(), "starting positions file to plot")
		("firstIndex,f", po::value<int>(), "index to plot from")
		("lastIndex,l", po::value<int>(), "index to plot to")
		("windowSize,w", po::value<int>(), "window size")
		("onlyLearnPErr,L", "print only region Learning Progress and Error plot")
		("onlyStartPosHis,H", "print only region Starting Position History plot")
	;
	
	po::variables_map vm;       
	po::store(po::parse_command_line(ac, av, desc), vm);
	po::notify(vm);   
	
	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}


	if (!vm.count("regFile") && !vm.count("stpFile")) {
		cout << "No source files specified. Exciting..." << endl;
		cout << "Use  --help  for argument options."  << endl;
		return 1;
	}


	if (vm.count("regFile")) {
		if (!region.read_data((vm["regFile"].as<string>()))) {
			cerr << "Specified reg file couldn't be read. Exiting..." << endl;
			return 1;
		} else {
			cout << endl << "reg file loaded." << endl;
		}
	}

	if (vm.count("stpFile")) {
		ifstream readFile ((vm["stpFile"].as<string>() + ".stp").c_str(), ios::in | ios::binary);
		if (!readFile) {
			cerr << "Specified stp file couldn't be read. Exiting..." << endl;
			return 1;
		} else {
			//read_realvector(readFile, experimentStartingPositions);
			read_vector<double>(readFile, experimentStartingPositions);
			cout << "stp file loaded." << endl;
		}
	}	

	if (vm.count("regFile")) {
		lastIndex = region.learningProgressHistory.size();
	} else {
		lastIndex = experimentStartingPositions.size();
	}


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
	


	if (windowSize > (lastIndex-firstIndex)) {
		windowSize = (lastIndex-firstIndex);
	}
	if (vm.count("windowSize")) {
		int ws = vm["windowSize"].as<int>();
		if (ws < 1 || ws > windowSize) {
				cerr << "Invalid window_size (out of range), using " << windowSize << " instead." << endl;		
		} else {
			windowSize = ws;
		}
	}

	cout << "Using " << firstIndex << " as first_index." << endl;
	cout << "Using " << lastIndex << " as last_index." << endl;
	cout << "Using " << windowSize << " as window_size." << endl;



	
	vector<vector<int> > statistics;
	string tempValuesName = "tempValues";	
	string tempScriptName = "tempgnuscript.gnu";

	//0: front, 1: back, 2: side, 3: zero action
	int dis = 3+1;
	int zero = dis-1;
	int group = 9+1;  // we need 9 / group to be 0
	


	if (vm.count("regFile") && !vm.count("onlyLearnPErr")) {

	statistics = vector<vector<int> >();

	plotting::orderStartPosHistoryValues(dis, &statistics, windowSize, /*(vector<int>*)*/&region.startingPositionsHistory, group, zero, firstIndex, lastIndex); 
	plotting::printStartPosHistoryValues(tempValuesName, &statistics);
	plotting::scriptValues((vm["regFile"].as<string>() + "-StartPos"), tempValuesName, tempScriptName, windowSize, region.index);
	plotting::plotAndFinish(tempValuesName, tempScriptName);
	
	cout << endl << "region starting positions history plot printed" << endl;
	}




	if (vm.count("regFile") && !vm.count("onlyStartPosHis")) {

	ofstream tempLProgErrFileValues(tempValuesName.c_str(), ios::out);
	for (int i = firstIndex; i < lastIndex; i++) {
		tempLProgErrFileValues << i << "\t";
		tempLProgErrFileValues << region.learningProgressHistory[i] << "\t";
		tempLProgErrFileValues << region.errorsHistory[i] << endl;
	}
	plotting::scriptValues((vm["regFile"].as<string>() + "-LProgErr"), tempValuesName, tempScriptName, windowSize, region.index, true);
	plotting::plotAndFinish(tempValuesName, tempScriptName);

	cout << endl << "region learning progress and error history plot printed" << endl;
	}


	
	if (vm.count("stpFile")) {

	statistics = vector<vector<int> >();

	plotting::orderStartPosHistoryValues(dis, &statistics, windowSize, &experimentStartingPositions, group, zero, firstIndex, lastIndex); 
	plotting::printStartPosHistoryValues(tempValuesName, &statistics);
	string StartPosgnuplotScriptName = "tempLStartPosgnuplotscript.gnu";
	plotting::scriptValues((vm["stpFile"].as<string>() + "-expStartPos"), tempValuesName, tempScriptName, windowSize);
	plotting::plotAndFinish(tempValuesName, tempScriptName);
	
	cout << endl << "experiment starting positions history plot printed" << endl;
	}


	
	cout << endl << "Plots printed, job done. Exiting..." << endl << endl;



} catch(std::exception& e) {
	cout << e.what() << "\n";
	cout << "Use  --help  for argument options."  << endl;
	
	return 1;
}
return 0;
}




