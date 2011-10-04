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

#include <string>
#include <iostream>

extern "C" {
#include <unistd.h>
#include <getopt.h>
}

using namespace std;

CommandLineAction
processCommandLineArgs(int argc, char ** argv, Settings & setup)
{
	bool help = false;
	setup.pcmCapture = false;
	setup.arg = "";

	static struct option longOptions[] = {
			{"help",       no_argument,       0, 'h'},
			{0, 0, 0, 0}
		};

	int idx = 0;

	while (1) {
		int c;

		c = getopt_long(argc, argv, "hn:e:", longOptions, &idx);
		if (c == -1) {
			break;
		}

		switch (c) {
		case 0:
			// flags
			break;

		case 'h':
			help = true;
			break;

		case '?':
			return Error;

		default:
			return Error;
		}
	}

//	cerr << "optind=" << optind << ", opterr=" << opterr << ", optopt=" << optopt << endl;

	if (help) {
		return PrintHelp;
	}
	else if (optind < argc) {
		string action = argv[optind];
		string arg;

		if (action == "") {
			cerr << PROGRAM_NAME << ": action required." << endl;
			return Error;
		}

		if (action == "list" || action == "ls") {
			return ListClients;
		}
		else if (action == "status" || action == "st") {
			return PrintStatus;
		}
		else if (action == "grammar" || action == "gr") {
			if (optind < argc - 1) {
				arg = argv[optind + 1];
				if (arg != "") {
					setup.arg = arg;
				}
				else {
					cerr << PROGRAM_NAME << ": grammar: requires an argument." << endl;
					return Error;
				}
			}
			else {
				cerr << PROGRAM_NAME << ": grammar: requires an argument." << endl;
				return Error;
			}
			return SetGrammar;
		}
		else if (action == "audiosource" || action == "as") {
			if (optind < argc - 1) {
				arg = argv[optind + 1];
				if (arg != "") {
					// set to a file
					setup.arg = arg;
				}
				else {
					cerr << PROGRAM_NAME << ": audiosource: requires an argument." << endl;
					return Error;
				}
			}
			else {
				// set to PCM capture
				setup.pcmCapture = true;
			}
			return SetAudioSource;
		}
		else if (action == "start") {
			return Start;
		}
		else if (action == "stop") {
			return Stop;
		}
		else if (action == "shutdown") {
			return Shutdown;
		}

		cerr << PROGRAM_NAME << ": unrecognised action `" << action << "'." << endl;
		return Error;
	}

	return Error;
}
