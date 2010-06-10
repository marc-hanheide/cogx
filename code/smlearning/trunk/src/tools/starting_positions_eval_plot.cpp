#include <metalearning/Scenario.h>
#include<tools/data_handling.h>


/*
plots the usage of different starting positions throughout the experiment
*/

//------------------------------------------------------------------------------

using namespace smlearning;

int main(int argc, char *argv[]) {


if (argc < 2) {
		cerr << argv[0] << " starting_positions_file (without extension .stp)" << endl;
		return 1;
	}	


	/////////////////////////////////////
	///// reading values


	string fileName = string (argv[1]);
	fileName += ".stp";
	ifstream readFile (fileName.c_str(), ios::in | ios::binary);
	if (!readFile) {
		cerr << "Specified file couldn't be read. Exiting..." << endl;
		return 1;
	}
	vector<int> startingPositions;
	read_intvector(readFile, startingPositions);
	

	//int startingPositionsCount = Scenario::getStartingPositionsCount();
	vector<vector<int> > statistics;

	//vector<int> startingPositions(startingPositionsCount+1, 0.0);	
	//0: front, 1: back, 2: side
	int dis = 3;
	int group = 9+1;
	vector<int> initial(dis, 0);
	statistics.push_back(initial);
	for (int i = 0; i < startingPositions.size() ; i++) {
		vector<int> newStat(statistics.back());
		newStat[startingPositions[i]/group]++;
		statistics.push_back(newStat);
	}


	string tempStartPositionsFileValuesName = "tempStartPositionsFileValues";
	ofstream tempStartPositionsFileValues(tempStartPositionsFileValuesName.c_str(), ios::out);
	for (int i = 0; i < statistics.size(); i++) {
		tempStartPositionsFileValues << i << "\t";
		for (int j = 0; j < statistics[0].size(); j++) {	
			tempStartPositionsFileValues << statistics[i][j] << "\t";
		}
		tempStartPositionsFileValues << endl;
	}


	stringstream actionsForStartPositionsGnuplotScript;

	actionsForStartPositionsGnuplotScript << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	actionsForStartPositionsGnuplotScript << "set output \"|epstopdf --filter > '" + fileName + "-StartPositions.pdf'" << endl;
	actionsForStartPositionsGnuplotScript << "set autoscale" << endl;
	actionsForStartPositionsGnuplotScript << "set style line 1 lt 1 lc rgb \"blue\"" << endl;
	actionsForStartPositionsGnuplotScript << "set style line 2 lt 1 lc rgb \"red\"" << endl;
	actionsForStartPositionsGnuplotScript << "set style line 3 lt 1 lc rgb \"green\"" << endl;
	// actionsForStartPositionsGnuplotScript << "set log y" << endl;
	actionsForStartPositionsGnuplotScript << "set title \" Usage of starting positions \"" << endl;
	actionsForStartPositionsGnuplotScript << "set xlabel \"Iterations\" " << endl;
	actionsForStartPositionsGnuplotScript << "set ylabel \"Values\" " << endl;
	actionsForStartPositionsGnuplotScript << "plot  \"" + tempStartPositionsFileValuesName + "\" "; 
	actionsForStartPositionsGnuplotScript << "using 1:2 ls 1 title 'starting positions 1 - 9' with lines, ";
	actionsForStartPositionsGnuplotScript << "\"" + tempStartPositionsFileValuesName + "\" "; 
	actionsForStartPositionsGnuplotScript << "using 1:3 ls 2 title 'starting positions 10 - 18' with lines, ";
	actionsForStartPositionsGnuplotScript << "\"" + tempStartPositionsFileValuesName + "\" "; 
	actionsForStartPositionsGnuplotScript << "using 1:4 ls 3 title 'starting positions 19 - 24' with lines" << endl;

	string StartPositionsgnuplotScriptName = "tempLStartPositionsgnuplotscript.gnu";

	ofstream StartPositionsgnuplotScript(StartPositionsgnuplotScriptName.c_str(), ios::out);
	StartPositionsgnuplotScript << actionsForStartPositionsGnuplotScript.str() << endl;
	StartPositionsgnuplotScript.close();

	


	/////////////////////////////////////
	///// executing scripts

	::system("gnuplot tempLStartPositionsgnuplotscript.gnu");
	


	/////////////////////////////////////
	///// removing temporary files
	
	::system(("rm " + tempStartPositionsFileValuesName).c_str());
	::system(("rm " + StartPositionsgnuplotScriptName).c_str());
	

	/////////////////////////////////////
	///// done

	cout << endl << "Plot printed, job done. Exiting..." << endl << endl;
	


	return 0;




}













