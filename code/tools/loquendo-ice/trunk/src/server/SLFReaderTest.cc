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

#include <iostream>
#include <cstdlib>

using namespace std;
using namespace ASR;
using namespace ASR::loquendo;

string
wordLatticeToString(WordLatticePtr wl)
{
	stringstream s(stringstream::in|stringstream::out);

	if (wl) {
		s << "nodes = {\n";
		vector<NodePtr>::iterator nit;
		for (nit = wl->nodes.begin(); nit != wl->nodes.end(); nit++) {
			s << "  [" << (*nit)->id << "]: t=" << (*nit)->time << ", f=" << (*nit)->frame << endl;
		}
		s << "}\n";

		s << "links = {\n";
		vector<LinkPtr>::iterator lit;
		for (lit = wl->links.begin(); lit != wl->links.end(); lit++) {
			s << "  (" << (*lit)->id << "): [" << (*lit)->start << "] -> [" << (*lit)->end << "]: \""
					<< (*lit)->word << "\" (c=" << (*lit)->confidence << ", w="
					<< (*lit)->searchScore << ", p=" << (*lit)->sequenceScore << ", a="
					<< (*lit)->acoustic << ")" << endl;
		}
		s << "}\n";
	}
	else {
		s << "NULL";
	}

	return s.str();
}

int
main(int argc, char ** argv)
{
	if (argc != 2) {
		cout << "Usage: " << argv[0] << " FILENAME" << endl;
		return EXIT_FAILURE;
	}
	string filename(argv[1]);

	cout << "Examining `" << filename << "'..." << endl;
	SLFReader reader;

	cout << wordLatticeToString(reader.readLatticeFromFile(filename)) << endl;

	return EXIT_SUCCESS;
}
