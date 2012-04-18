/** @file data_handling.cpp
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

#include <smltools/helpers.h>
#include <boost/regex.hpp>

namespace smlearning {


///
///returns file name based on current time
///
string get_base_filename_from_time () {
	time_t rawtime;
	struct tm * timeinfo;
  	char buffer [12];

 	time ( &rawtime );
  	timeinfo = localtime ( &rawtime );

  	strftime (buffer,12,"%y%m%d%H%M",timeinfo);
  	puts(buffer);

	string name;
	name.append(buffer);

	return name;

}


///
///a utility function to obtain the file name from a possibly large path/file pattern
///
string get_seqBaseFileName (string seqFile) {
	boost::regex seqfile_re (".*/(.*)$");
	boost::cmatch match;
	string seqBaseFileName;
	if (boost::regex_match(seqFile.c_str(), match, seqfile_re)) {
		seqBaseFileName = string(match[1].first, match[1].second);
	}
	else
		seqBaseFileName = seqFile;
	cout << "seqBaseFileName: " << seqBaseFileName << endl;
	cout << "seqFile: " << seqFile << endl;
	return seqBaseFileName;
}



///
///parse a string containing a list of starting positions (TODO: write a nice reg.exp.)
///
vector<int> parse_startingPositions(string argsStr, int maxStartPos) {
	
	vector<int> positions;
	int currNumber1 = 0;
	int currNumber2 = 0;
	bool range = false;



	string::iterator it;
	for ( it=argsStr.begin() ; it < argsStr.end(); it++ ) {
		const char character = *it;
		// cout << "character "  << character << endl;
		if (isdigit(character) && !range) {
			currNumber1 *= 10;
			currNumber1 += atoi(&character);
			// cout << "case 1: new nr1: " << currNumber1 << endl;
		} else if (isdigit(character) && range) {
			currNumber2 *= 10;
			currNumber2 += atoi(&character);
			// cout << "case 2: new nr2: " << currNumber2 << endl;
		} else if (character == "-"[0]) {
			// cout << "case 3: dash" << endl;
			range = true;
		} else  if (character == ","[0]) {
			// cout << "case 4: comma" << endl;
			if (currNumber1 > 0 && currNumber1 <= maxStartPos) {
				// cout << "appending nr1 :" << currNumber1 << endl;
			positions.push_back(currNumber1);
			}
			if (range) {
				// cout << "range on" << endl;
				for (int i = (currNumber1 + 1); i < currNumber2; i++) {
					if (i > 0 && i <= maxStartPos) {
					positions.push_back(i);
					// cout << "appending :" << i << endl;
					}
				}
				if (currNumber1 > 0 && currNumber2 <= maxStartPos) {
					// cout << "appending nr2 :" << currNumber2 << endl;
				positions.push_back(currNumber2);			
				}
			}
			currNumber1 = 0;
			currNumber2 = 0;
			range = false;
// cout << "variables set back: " << currNumber1 << " " << currNumber2 << " " << range << endl;
		} else {
			// cout << "nothing" << endl;
		} 
	}
	if (argsStr.at(argsStr.length()-1) != ","[0]) {
			if (currNumber1 > 0 && currNumber1 <= maxStartPos) {
				// cout << "appending nr1 :" << currNumber1 << endl;
			positions.push_back(currNumber1);
			}
			if (range) {
				// cout << "range on" << endl;
				for (int i = (currNumber1 + 1); i < currNumber2; i++) {
					if (i > 0 && i <= maxStartPos) {
					positions.push_back(i);
					// cout << "appending :" << i << endl;
					}
				}
				if (currNumber1 > 0 && currNumber2 <= maxStartPos) {
					// cout << "appending nr2 :" << currNumber2 << endl;
				positions.push_back(currNumber2);			
				}
			}
	}
	// cout << "finished" << endl;
	return positions;
}




};  /* smlearning namespace */
