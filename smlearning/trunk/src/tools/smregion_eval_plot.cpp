#include <metalearning/Scenario.h>
#include <metalearning/SMRegion.h>



/*

smregion_eval_plot becomes a name of a .reg file to read, reads it and plots the learning progress data and error data into a pdf file. This file has the same name as the argument file and is stored in the working directory.

Optionally one can specify also the first index or the first and the last index. Data between the first and last index will be plotted.

*/

//------------------------------------------------------------------------------

using namespace smlearning;

int main(int argc, char *argv[]) {

	string correctArgumentFormating = " region_file (without the .reg extension) [ -fi first_index] [-li last_index] [-ws window_size]";

	if (argc < 2) {
		cerr << argv[0] << correctArgumentFormating << endl;
		return 1;
	}	


	/////////////////////////////////////
	///// reading values


	string fileName = string (argv[1]);

	SMRegion region;
	if (!region.read_data(fileName)) {
		cerr << "Specified file couldn't be read. Exiting..." << endl;
		return 1;
	}
	

	int firstIndex = 0;
	int lastIndex = region.learningProgressHistory.size();
	int windowSize = 20;

	region.print_data ();
	cout << endl;

	if (windowSize >= region.startingPositionsHistory.size()) {
		windowSize = region.startingPositionsHistory.size() -1;
	}
	if (region.startingPositionsHistory.size() < 1) {
		cerr << "startingPositionsHistory does not contain any values, setting window_size = 0." << endl;
		windowSize = 0;
	}


	if (argc > 3) {
	for (int currentArgument = 2; currentArgument < argc; currentArgument +=2) {
		if (string (argv[currentArgument]) == "-fi") {
			if (atoi(argv[currentArgument+1]) < firstIndex || atoi(argv[currentArgument+1]) > lastIndex) {
				cerr << "Invalid first_index (out of range), using " << firstIndex << " instead." << endl;		
			} else {
				firstIndex = atoi(argv[currentArgument+1]);
			}
		} else if (string (argv[currentArgument]) == "-li") {
			if (atoi(argv[currentArgument+1]) < firstIndex || atoi(argv[currentArgument+1]) > lastIndex) {
				cerr << "Invalid last_index (out of range), using " << lastIndex << " instead." << endl;		
			} else {
				lastIndex = atoi(argv[currentArgument+1]);
			}
		} else if (string (argv[currentArgument]) == "-ws") {
			if (atoi(argv[currentArgument+1]) < 1 || atoi(argv[currentArgument+1]) >= region.startingPositionsHistory.size()) {
				cerr << "Invalid window_size (out of range), using " << windowSize << " instead." << endl;
			} else {
				windowSize = atoi(argv[currentArgument+1]);
			}
		} else {
			cerr << "Invalid argument formating. Use " << correctArgumentFormating << endl;
		}
	}
	}// else {
	//	cerr << "Invalid argument formating. Use " << correctArgumentFormating << endl;
	//}
	cout << "Using " << firstIndex << " as first_index." << endl;
	cout << "Using " << lastIndex << " as last_index." << endl;
	cout << "Using " << windowSize << " as window_size." << endl;
	cout << "Region created in iteration nr. " << region.createdInIteration << endl;	


/*

	if (argc > 2) {
		if (atoi(argv[2]) < firstIndex || atoi(argv[2]) > lastIndex) {
			cerr << "Invalid first_index (out of range), using " << firstIndex << " instead." << endl;		
		} else {
			firstIndex = atoi(argv[2]);
		}
	}
	cout << "Using " << firstIndex << " as first_index." << endl;
	if (argc > 3) {
		if (atoi(argv[3]) < firstIndex || atoi(argv[3]) > lastIndex) {
			cerr << "Invalid last_index (out of range), using " << lastIndex << " instead." << endl;		
		} else {
			lastIndex = atoi(argv[3]);
		}
	}
	cout << "Using " << lastIndex << " as last_index." << endl;

*/


	

	/////////////////////////////////////
	///// preparing start position history for writing to a file

		
	//int startingPositionsCount = Scenario::getStartingPositionsCount();
	vector<vector<int> > statistics;

	//vector<int> startingPositions(startingPositionsCount+1, 0.0);	
	//0: front, 1: back, 2: side, 3: zero action
	int dis = 3+1;
	int zero = dis-1;
	int group = 9+1;  // we need 9 / group to be 0
	vector<int> startingPositions(dis, 0);
	statistics.push_back(startingPositions);
	for (int i = 0; i < windowSize; i++) {
		if (region.startingPositionsHistory[i] != 0) {
			startingPositions[(region.startingPositionsHistory[i]/group)]++;
		} else {
			startingPositions[zero]++;
		}
		
//cout << endl;
//cout << region.startingPositionsHistory[i] << endl;
//cout << startingPositions[(region.startingPositionsHistory[i]/group)] << endl;
	}
	
	statistics.push_back(startingPositions);	

//for (int i = 0; i < startingPositions.size(); i++) {
//cout << "  " << startingPositions[i];
//}
//cout << endl;
//cout << endl;

		
	for (int j = windowSize; j < region.startingPositionsHistory.size(); j++){
		vector<int> newStat(statistics.back());
		//newStat[(int)(region.startingPositionsHistory[j-startingPositionsCount])]--;
		if (region.startingPositionsHistory[(j-windowSize)] != 0) {
			newStat[(region.startingPositionsHistory[(j-windowSize)]/group)]--;	
		} else {
			newStat[zero]--;
		}
		if (region.startingPositionsHistory[j] != 0) {
			newStat[(region.startingPositionsHistory[j]/group)]++;
		}  else {
			newStat[zero]++;
		}
		statistics.push_back(newStat);
	}
//for (int i = 0; i < statistics.size(); i++) {
//for (int j = 0; j < startingPositions.size(); j++) {
//cout << "  " << statistics[i][j];
//}
//cout << endl;
//}




	/////////////////////////////////////
	///// writing files with values to plot

	string tempLProgErrFileValuesName = "tempLProgErrFileValues";
	ofstream tempLProgErrFileValues(tempLProgErrFileValuesName.c_str(), ios::out);
	for (int i = firstIndex; i < lastIndex; i++) {
		tempLProgErrFileValues << i+region.createdInIteration << "\t";
		tempLProgErrFileValues << region.learningProgressHistory[i] << "\t";
		tempLProgErrFileValues << region.errorsHistory[i] << endl;
	}
	

	string tempStartPosFileValuesName = "tempStartPosFileValues";
	ofstream tempStartPosFileValues(tempStartPosFileValuesName.c_str(), ios::out);
	for (int i = 1; i < statistics.size(); i++) {
		tempStartPosFileValues << (i-1)+region.createdInIteration << "\t";
		for (int j = 0; j < statistics[0].size(); j++) {	
			tempStartPosFileValues << statistics[i][j] << "\t";
		}
		tempStartPosFileValues << endl;
	}




	/////////////////////////////////////
	///// preparing scripts

	stringstream actionsForLProgErrGnuplotScript;

	actionsForLProgErrGnuplotScript << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	actionsForLProgErrGnuplotScript << "set output \"|epstopdf --filter > '" + fileName + "-LProgErr.pdf'" << endl;
	actionsForLProgErrGnuplotScript << "set autoscale" << endl;
	actionsForLProgErrGnuplotScript << "set style line 1 lt 1 lc rgb \"blue\"" << endl;
	actionsForLProgErrGnuplotScript << "set style line 2 lt 1 lc rgb \"red\"" << endl;
	// actionsForLProgErrGnuplotScript << "set log y" << endl;
	actionsForLProgErrGnuplotScript << "set title \"Region " << region.index << "\"" << endl;
	actionsForLProgErrGnuplotScript << "set xlabel \"Iterations\" " << endl;
	actionsForLProgErrGnuplotScript << "set ylabel \"Values\" " << endl;
	actionsForLProgErrGnuplotScript << "plot  \"" + tempLProgErrFileValuesName + "\" "; 
	actionsForLProgErrGnuplotScript << "using 1:2 ls 1 title 'Learning progress history' with lines, ";
	actionsForLProgErrGnuplotScript << "\"" + tempLProgErrFileValuesName + "\" "; 
	actionsForLProgErrGnuplotScript << "using 1:3 ls 2 title 'Errors history' with lines" << endl;

	string LProgErrgnuplotScriptName = "tempLProgErrgnuplotscript.gnu";

	ofstream LProgErrgnuplotScript(LProgErrgnuplotScriptName.c_str(), ios::out);
	LProgErrgnuplotScript << actionsForLProgErrGnuplotScript.str() << endl;
	LProgErrgnuplotScript.close();



	stringstream actionsForStartPosGnuplotScript;

	actionsForStartPosGnuplotScript << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	actionsForStartPosGnuplotScript << "set output \"|epstopdf --filter > '" + fileName + "-StartPos.pdf'" << endl;
	actionsForStartPosGnuplotScript << "set autoscale" << endl;
	actionsForStartPosGnuplotScript << "set style line 1 lt 1 lc rgb \"blue\"" << endl;
	actionsForStartPosGnuplotScript << "set style line 2 lt 1 lc rgb \"red\"" << endl;
	actionsForStartPosGnuplotScript << "set style line 3 lt 1 lc rgb \"green\"" << endl;
	actionsForStartPosGnuplotScript << "set style line 4 lt 1 lc rgb \"yellow\"" << endl;
	// actionsForStartPosGnuplotScript << "set log y" << endl;
	actionsForStartPosGnuplotScript << "set title \"Region " << region.index << " (window size: " << windowSize << ")\"" << endl;
	actionsForStartPosGnuplotScript << "set xlabel \"Iterations\" " << endl;
	actionsForStartPosGnuplotScript << "set ylabel \"Values\" " << endl;
	actionsForStartPosGnuplotScript << "plot  \"" + tempStartPosFileValuesName + "\" "; 
	actionsForStartPosGnuplotScript << "using 1:2 ls 1 title 'starting positions 1 - 9' with lines, ";
	actionsForStartPosGnuplotScript << "\"" + tempStartPosFileValuesName + "\" "; 
	actionsForStartPosGnuplotScript << "using 1:3 ls 2 title 'starting positions 10 - 18' with lines, ";
	actionsForStartPosGnuplotScript << "\"" + tempStartPosFileValuesName + "\" "; 
	actionsForStartPosGnuplotScript << "using 1:4 ls 3 title 'starting positions 19 - 24' with lines, ";
	actionsForStartPosGnuplotScript << "\"" + tempStartPosFileValuesName + "\" "; 
	actionsForStartPosGnuplotScript << "using 1:5 ls 4 title 'region not used' with lines" << endl;

	string StartPosgnuplotScriptName = "tempLStartPosgnuplotscript.gnu";

	ofstream StartPosgnuplotScript(StartPosgnuplotScriptName.c_str(), ios::out);
	StartPosgnuplotScript << actionsForStartPosGnuplotScript.str() << endl;
	StartPosgnuplotScript.close();





	/////////////////////////////////////
	///// executing scripts

//	string command ("gnuplot gnuplotscript.gnu");
//	::system(command.c_str());
	::system("gnuplot tempLProgErrgnuplotscript.gnu");
	::system("gnuplot tempLStartPosgnuplotscript.gnu");
	


	/////////////////////////////////////
	///// removing temporary files
	::system(("rm " + tempLProgErrFileValuesName).c_str());
	::system(("rm " + LProgErrgnuplotScriptName).c_str());
	
	::system(("rm " + tempStartPosFileValuesName).c_str());
	::system(("rm " + StartPosgnuplotScriptName).c_str());
	

	/////////////////////////////////////
	///// done

	cout << endl << "Plots printed, job done. Exiting..." << endl << endl;
	


	return 0;
}
