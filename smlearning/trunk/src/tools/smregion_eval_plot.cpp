#include <metalearning/SMRegion.h>

/*

pdf_plotter becomes a name of a .reg file to read, reads it and plots the learning progress data and error data into a pdf file. This file has the same name as the argument file and is stored in the working directory.

Optionally one can specify also the first index or the first and the last index. Data between the first and last index will be plotted.

*/

//------------------------------------------------------------------------------

using namespace smlearning;

int main(int argc, char *argv[]) {
	if (argc < 2) {
		cerr << argv[0] << " region_file (without the .reg extension) [first_index] [last_index]" << endl;
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



	/////////////////////////////////////
	///// preparing file with values to plot

	string tempFileValuesName = "tempFileValues";
	ofstream tempFileValues(tempFileValuesName.c_str(), ios::out);
	for (int i = firstIndex; i < lastIndex; i++) {
		tempFileValues << i << "\t";
		tempFileValues << region.learningProgressHistory[i] << "\t";
		tempFileValues << region.errorsHistory[i] << endl;
	}
	
	/////////////////////////////////////
	///// preparing script

	stringstream actionsForGnuplotScript;

	actionsForGnuplotScript << "set terminal postscript eps enhanced color font \"Times-Roman,14\"" << endl;
	actionsForGnuplotScript << "set output \"|epstopdf --filter > '" + fileName + ".pdf'" << endl;
	actionsForGnuplotScript << "set style line 1 lt 1 lc rgb \"red\"" << endl;
	actionsForGnuplotScript << "set style line 2 lt 1 lc rgb \"blue\"" << endl;
	// actionsForGnuplotScript << "set log y" << endl;
	actionsForGnuplotScript << "set title \"Region " << region.index << "\"" << endl;
	actionsForGnuplotScript << "plot  \"" + tempFileValuesName + "\" "; 
	actionsForGnuplotScript << "using 1:2 ls 1 title 'Learning progress history' with lines, ";
	actionsForGnuplotScript << "\"" + tempFileValuesName + "\" "; 
	actionsForGnuplotScript << "using 1:3 ls 2 title 'Errors history' with lines" << endl;

	string gnuplotScriptName = "tempgnuplotscript.gnu";

	ofstream gnuplotScript(gnuplotScriptName.c_str(), ios::out);
	gnuplotScript << actionsForGnuplotScript.str() << endl;
	gnuplotScript.close();

	/////////////////////////////////////
	///// executing script

//	string command ("gnuplot gnuplotscript.gnu");
//	::system(command.c_str());
	::system("gnuplot tempgnuplotscript.gnu");


	/////////////////////////////////////
	///// removing temporary files
	::system(("rm " + tempFileValuesName).c_str());
	::system(("rm " + gnuplotScriptName).c_str());
	
	

	


	return 0;
}
