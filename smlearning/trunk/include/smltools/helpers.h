/** @file data_handling.h
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa
 
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

#ifndef SMLEARNING_HELPERS_H_
#define SMLEARNING_HELPERS_H_

#include <cstdlib>
#include <ctime>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>
#include <boost/regex.hpp>


using namespace std;

namespace smlearning {

///
///Floating point representation for a feature vector.
///This representation should also allow for labels, i.e., a vector of size 1
///properly discretized
///
typedef vector<double> FeatureVector;

/**
   \enum write_mode
   \brief either binary or text files handling
*/
enum write_mode {
	_binary,
	_text
};

///
///function that prints the passed argument
///
template <typename T>
void print_item (const T& elem) {
    cout << elem << " ";
}

///
///print a FeatureVector struct
///
template <typename T>
void print_featvector (const vector<T>& v) {

	for_each (v.begin(), v.end(), print_item<T>);

}

///
///write a vector to a file
///
template <typename T>
void write_vector (ostream& writeFile, const vector<T>& v, unsigned int write_mode = _binary) {
	if (write_mode == _binary) {
		long featvectorSize = v.size();
		writeFile.write ((const char*)&featvectorSize, sizeof (v.size()));
		typename vector<T>::const_iterator n;
		for (n=v.begin(); n!= v.end(); n++) {
			writeFile.write ((const char* )&(*n), sizeof (*n));
		}
	}
	else if (write_mode == _text) {
		typename vector<T>::const_iterator n;
		for (n=v.begin(); n!= v.end(); n++) {
			writeFile << *n << "  ";
		}		
	}
}



///
///read a vector to a file
///
template <typename T>
void read_vector (ifstream& readFile, vector<T>& v) {
	long featvectorSize;
	readFile.read ((char *)&featvectorSize, sizeof(featvectorSize));
	// cout << "\t\t" << featvectorSize << endl;
	for (int n=0; n<featvectorSize; n++) {
		T value;
		readFile.read ((char* )&value, sizeof(value));
		v.push_back (value);
	}
}

///
///returns file name based on current time
///
string get_base_filename_from_time ();

///
///a utility function to obtain the file name from a possibly large path/file pattern
///
string get_seqBaseFileName (string seqFile);

///
///parse a string containing a list of starting positions (TODO: write a nice reg.exp.)
///
vector<int> parse_startingPositions(string str, int maxStartPos);

}; /* smlearning namespace */

#endif /* SMLEARNING_DATAHANDLING_H_*/
