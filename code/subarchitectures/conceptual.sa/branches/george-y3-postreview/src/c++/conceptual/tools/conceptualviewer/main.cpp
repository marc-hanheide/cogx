#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <sstream>
#include <dai/factorgraph.h>

using namespace dai;
using namespace std;

int main( int argc, char *argv[] )
{
    if( argc != 4 )
    {
        cout << "Usage: ./conceptualviewer <conceptual.fg> <conceptual.var> <output.out>" << endl << endl;
        return 1;
    }

	char *graphFile = argv[1];
	char *varsFile = argv[2];

	// Read factorgraph
	FactorGraph fg;
	fg.ReadFromFile( graphFile );

	// Read the variable file
	map<int, string> _varLabelToName;
	map<int, string> _factorLabelToName;
	ifstream varsStr( varsFile );
	while (varsStr.good())
	{
		string line;
	    getline(varsStr, line);
	    if (line.length()>0)
	    {
	    	if (line[0]=='x')
	    	{
	    		istringstream iss(line);
	    		string varLabel;
	    		iss>>varLabel;
	    		string varName;
	    		iss>>varName;
	    		const char *tmp = varLabel.c_str();
	    		_varLabelToName[boost::lexical_cast<int>(tmp+1)] = varName;
	    	}
	    	if (line[0]=='f')
	    	{
	    		istringstream iss(line);
	    		string varLabel;
	    		iss>>varLabel;
	    		string varName;
	    		iss>>varName;
	    		const char *tmp = varLabel.c_str();
	    		 replace(varName.begin(), varName.end(), '(', '_');
	    		 replace(varName.begin(), varName.end(), ')', '_');
	    		 replace(varName.begin(), varName.end(), ',', '_');
	    		_factorLabelToName[boost::lexical_cast<int>(tmp+1)] = varName + "_" + varLabel;
	    	}
	    }
	}
	varsStr.close();

	// Open output file for writing (except if filename equals "-")
	ostream *os = &cout;
	ofstream outfile;
	if( string( argv[3] ) != "-" ) {
		outfile.open( argv[3] );
		if( !outfile.is_open() ) {
			cerr << "Cannot open " << argv[3] << " for writing" << endl;
			return 1;
		}
		os = &outfile;
	} // else, write to cout

	// Write the factor graph
    (*os) << "graph FactorGraph { overlap=scalexy splines=true" << endl;
    (*os) << "node[shape=ellipse];" << endl;
    for( size_t i = 0; i < fg.nrVars(); i++ )
    {
    	// Get name of the variable
    	(*os) << "\t" << _varLabelToName[fg.var(i).label()] << ";" << endl;
    }
    (*os) << "node[shape=box];" << endl;
    for( size_t I = 0; I < fg.nrFactors(); I++ )
    	(*os) << "\t" << _factorLabelToName[I] << ";" << endl;
    for( size_t i = 0; i < fg.nrVars(); i++ )
        foreach( const Neighbor &I, fg.nbV(i) )  // for all neighboring factors I of i
          (*os) << "\t" << _varLabelToName[fg.var(i).label()] << " -- " << _factorLabelToName[I] << ";" << endl;
    (*os) << "}" << endl;

	return 0;
}
