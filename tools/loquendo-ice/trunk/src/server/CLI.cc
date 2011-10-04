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

#include "CLI.h"

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

CommandLineAction
processCommandLineArgs(int argc, char ** argv, Settings & setup)
{
	bool printHelp = false;
	bool printVersion = false;

	static struct option longOptions[] = {
			{"help",           no_argument,       0, 'h'},
			{"version",        no_argument,       0, 'v'},
			{"name",           required_argument, 0, 'n'},
			{"endpoints",      required_argument, 0, 'e'},
			{"session",        required_argument, 0, 's'},
			{"grammar",        required_argument, 0, 'g'},
			{"dump",           required_argument, 0, 'd'},
			{"word-lattices",  no_argument,       0, 'w'},
			{0, 0, 0, 0}
		};

	while (1) {
		int c;
		int idx = 0;

		c = getopt_long(argc, argv, "hvn:e:s:g:d:w", longOptions, &idx);
		if (c == -1) {
			break;
		}

		switch (c) {
		case 0:
			// flags
			break;

		case 'h':
			printHelp = true;
			break;

		case 'v':
			printVersion = true;
			break;

		case 'n':
			setup.serverName = optarg;
			break;

		case 'e':
			setup.serverEndpoints = optarg;
			break;

		case 's':
			setup.sessionFile = optarg;
			break;

		case 'g':
			setup.grammarFile = optarg;
			break;

		case 'd':
			setup.audiodumpDir = optarg;
			break;

		case 'w':
			setup.exportWordLattices = true;
			break;

		case '?':
			return Error;

		default:
			return Error;
		}
	}

	return (printHelp ? PrintHelp : (printVersion ? PrintVersion : Start));
}
