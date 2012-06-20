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

#include <cstdlib>

#include <iostream>
#include <Ice/Ice.h>
#include <IceUtil/IceUtil.h>
#include <IceUtil/CtrlCHandler.h>

#include "asr-loquendo.h"

#include "CLI.h"

namespace LoqASR = ::de::dfki::lt::tr::dialogue::asr::loquendo;

using namespace std;

const string PRX_STRING_ENVVAR = "LOQASR_PRX";
const string DEFAULT_PRX_STRING = "LoquendoASRServer:tcp -p 9021";

//------------------------------------------------------------------------------

void
printUsage();

int
workWithServer(const string & prx_string, CommandLineAction action, const Settings & setup);

//------------------------------------------------------------------------------

int
main(int argc, char ** argv)
{
	Settings setup;
	const char * env_prx_string = getenv(PRX_STRING_ENVVAR.c_str());
	string prx_string(env_prx_string == NULL ? DEFAULT_PRX_STRING : env_prx_string);

	CommandLineAction action = processCommandLineArgs(argc, argv, setup);

	switch (action) {
	case PrintHelp:
		printUsage();
		return EXIT_SUCCESS;

	case Error:
		cerr << PROGRAM_NAME << ": usage error, type `" << PROGRAM_NAME << " -h' for help." << endl;
		return EXIT_FAILURE;

	default:
		return workWithServer(prx_string, action, setup);
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
printUsage()
{
	cout << "Usage: " << PROGRAM_NAME << " [OPTIONS] ACTION" << endl
		<< endl
		<< "Where OPTIONS may be one or more of the following:" << endl
		<< "  -h, -help       ... print usage information" << endl
		<< endl
		<< "ACTION may be one of the following:" << endl
		<< "  " << ACTION_PrintStatus << "  ... print the server status string" << endl
		<< "  " << ACTION_ListClients << "  ... list clients attached to the server" << endl
		<< "  " << ACTION_SetGrammar << " FILE  ... set grammar file to FILE" << endl
		<< "  " << ACTION_SetAudioSource << "  ... set audio source to PulseAudio PCM capture" << endl
		<< "  " << ACTION_SetAudioSource << " FILE  ... set audio source to the FILE" << endl
		<< "  " << ACTION_Start << "  ... start recognition" << endl
		<< "  " << ACTION_Stop << "  ... stop recognition" << endl
		<< "  " << ACTION_Shutdown << "  ... shut down the server" << endl
		<< endl
		<< "The environment variable " << PRX_STRING_ENVVAR << " is used to determine the name and endpoints" << endl
		<< "of the ASR server. When unset, the following default value will be used:" << endl
		<< endl
		<< "    " << PRX_STRING_ENVVAR << "=\"" << DEFAULT_PRX_STRING << "\"" << endl;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

LoqASR::ServerPrx
getConnectionToServer(const string & prx_string, const Settings & setup)
{
	Ice::CommunicatorPtr ic = Ice::initialize();
	try {
		// connect to the server
		Ice::ObjectPrx base = ic->stringToProxy(prx_string);
		LoqASR::ServerPrx asr = LoqASR::ServerPrx::checkedCast(base);
		return asr;
	}
	catch (const LoqASR::LoquendoException & ex) {
		cerr << ex.function << " (" << ex.returnCode << "): " << ex.errorMessage << endl;
	}
	catch (const Ice::Exception& ex) {
		cerr << "Ice exception: " << ex << endl;
	}
	return 0;
}


int
workWithServer(const string & prx_string, CommandLineAction action, const Settings & setup)
{
	int status = 0;

	LoqASR::ServerPrx srv = getConnectionToServer(prx_string, setup);
	if (!srv) {
		cerr << "Unable to connect to the server at \"" << prx_string << "\"." << endl;
		return -1;
	}

	switch (action) {

	case PrintStatus:
		cout << srv->getStatusString();
		break;

	case ListClients:
		{
			vector<Ice::Identity> clients = srv->getClients();
			for (vector<Ice::Identity>::iterator it = clients.begin(); it != clients.end(); it++) {
				cout << it->name << endl;
			}
		}
		break;

	case SetGrammar:
		srv->setGrammarFile(setup.arg);
		break;

	case SetAudioSource:
		{
			LoqASR::AudioSourcePtr as = 0;
			if (setup.pcmCapture) {
				as = new LoqASR::PulseAudioPCMCapture();
			}
			else {
				as = new LoqASR::RAWFile(setup.arg);
			}
			srv->setAudioSource(as);
		}
		break;

	case Start:
		srv->start();
		break;

	case Stop:
		srv->stop();
		break;

	case Shutdown:
		srv->shutdown();
		break;

	default:
		cerr << "Error: action unimplemented" << endl;
		status = 1;
		break;
	}

	return status;
}
