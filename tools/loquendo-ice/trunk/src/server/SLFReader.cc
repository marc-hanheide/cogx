// ----------------------------------------------------------------------------
// Copyright (C) 2010-2011 DFKI GmbH Talking Robots 
// Miroslav Janicek (miroslav.janicek@dfki.de) 
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// ----------------------------------------------------------------------------

#include "SLFReader.h"

#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;
using namespace boost;
using namespace LoqASR;
using namespace LoqASR::result;

const string SILENCE_FROM = "rumore|\\d1";
const string SILENCE_TO = "<sil>";

SLFReader::SLFReader()
{ }

SLFReader::~SLFReader()
{ }

WordLatticePtr
SLFReader::readLatticeFromFile(const string & filename)
{
	ifstream in(filename.c_str());
	if (!in.is_open()) return 0;

	SLFLine sl;

	if (!readSLFLine(in, sl)) {
		cout << "ERROR: expected file header" << endl;
		return 0;  // TODO: throw an exception
	}

	int num_nodes = boost::lexical_cast<int>(sl["N"]);
	int num_links = boost::lexical_cast<int>(sl["L"]);

	WordLatticePtr wl = new WordLattice();

	for (int i = 0; i < num_nodes; i++) {
		if (!readSLFLine(in, sl)) {
			cout << "ERROR: expected a node definition" << endl;
			return 0;  // TODO: throw an exception
		}
		wl->nodes.push_back(readNodeFromSLFLine(sl));
	}

	for (int i = 0; i < num_links; i++) {
		if (!readSLFLine(in, sl)) {
			cout << "ERROR: expected a link definition" << endl;
			return 0;  // TODO: throw an exception
		}
		wl->links.push_back(readLinkFromSLFLine(sl));
	}

	in.close();

	return wl;
}

NodePtr
SLFReader::readNodeFromSLFLine(SLFLine & l)
{
	NodePtr node = new Node();
	node->id = boost::lexical_cast<int>(l["I"]);
	node->time = boost::lexical_cast<float>(l["t"]);
	node->frame = boost::lexical_cast<float>(l["f"]);
	return node;
}

LinkPtr
SLFReader::readLinkFromSLFLine(SLFLine & l)
{
	LinkPtr link = new Link();
	string w = l["W"].substr(1, l["W"].size() - 2);

	link->id = boost::lexical_cast<int>(l["J"]);
	link->start = boost::lexical_cast<int>(l["S"]);
	link->end = boost::lexical_cast<int>(l["E"]);
	link->word = (w == SILENCE_FROM ? SILENCE_TO : w);
	link->confidence = boost::lexical_cast<float>(l["c"]);
	link->searchScore = boost::lexical_cast<float>(l["w"]);
	link->sequenceScore = boost::lexical_cast<float>(l["p"]);
	link->acoustic = boost::lexical_cast<float>(l["a"]);
	return link;
}

bool
SLFReader::readSLFLine(istream & in, SLFLine & sl)
{
	string line;
	vector<string> vec;

	typedef tokenizer<char_separator<char> > Tokenizer;
	char_separator<char> field_sep(" ");
	char_separator<char> item_sep("=");

	sl.clear(); 

	while (getline(in, line))
	{
		if (line[0] != '#') {
			Tokenizer tok(line, field_sep);
			vec.assign(tok.begin(), tok.end());

			vector<string> col;

			for (vector<string>::iterator it = vec.begin(); it != vec.end(); it++) {
				Tokenizer ctok(*it, item_sep);
				col.assign(ctok.begin(), ctok.end());

				if (col.size() == 2) {
					sl[col[0]] = col[1];
				}
				else {
					// formatting error exception
					return false;
				}
			}
			return true;
		}
	}
	return false;
}

