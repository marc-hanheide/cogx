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
#include "MonitorListener.h"

#include "CLI.h"
#include "TtyUtils.h"

using namespace std;

const string PRX_STRING_ENVVAR = "LOQASR_PRX";
const string DEFAULT_PRX_STRING = "LoquendoASRServer:tcp -p 9021";

//------------------------------------------------------------------------------

void
printUsage();

int
registerWithServer(const string & prx_string, const Settings & setup);

//------------------------------------------------------------------------------

int
main(int argc, char ** argv)
{
	Settings setup;
	const char * env_prx_string = getenv(PRX_STRING_ENVVAR.c_str());
	string prx_string(env_prx_string == NULL ? DEFAULT_PRX_STRING : env_prx_string);

	switch (processCommandLineArgs(argc, argv, setup)) {
	case PrintHelp:
		printUsage();
		return EXIT_SUCCESS;

	case Register:
		return registerWithServer(prx_string, setup);

	case Error:
	default:
		cerr << PROGRAM_NAME << ": usage error, type `" << PROGRAM_NAME << " -h' for help." << endl;
		return EXIT_FAILURE;
	}
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
printUsage()
{
	cout << "Usage: " << PROGRAM_NAME << " [OPTIONS]" << endl
		<< endl
		<< "Where OPTIONS may be one or more of the following:" << endl
		<< "  -h, -help       ... print (this) usage information" << endl
		<< endl
		<< "  -s              ... start recognition once registered" << endl
		<< endl
		<< "The environment variable " << PRX_STRING_ENVVAR << " is used to determine the name and endpoints" << endl
		<< "of the ASR server. When unset, the following default value will be used:" << endl
		<< endl
		<< "    " << PRX_STRING_ENVVAR << "=\"" << DEFAULT_PRX_STRING << "\"" << endl;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int
registerWithServer(const string & prx_string, const Settings & setup)
{
	cerr << tty::yellow << "* connecting to \"" << prx_string << "\"" << tty::dcol << endl;
	int status = 0;

	Ice::PropertiesPtr props = Ice::createProperties();
	props->setProperty("Ice.ACM.Client", "0");
	Ice::InitializationData id;
	id.properties = props;

	Ice::CommunicatorPtr ic = Ice::initialize(id);
	try {
		// connect to the server
		Ice::ObjectPrx base = ic->stringToProxy(prx_string);
		LoqASR::ServerPrx asr = LoqASR::ServerPrx::checkedCast(base);
		if (!asr) {
			throw "Invalid proxy";
		}

		// now create and register the callback
		Ice::ObjectAdapterPtr adapter = ic->createObjectAdapter("");
		Ice::Identity ident;
		ident.name = IceUtil::generateUUID();
		ident.category = "";
		LoqASR::ClientPtr consumer = new MonitorListener(ic);
		adapter->add(consumer, ident);
		adapter->activate();

		asr->ice_getConnection()->setAdapter(adapter);
		asr->addClient(ident);
		if (setup.start) {
			asr->start();
		}
		cerr << tty::green << "* ready, waiting for shutdown" << tty::dcol << endl;
		ic->waitForShutdown();
	}
	catch (const LoqASR::LoquendoException & ex) {
		cerr << ex.function << " (" << ex.returnCode << "): " << ex.errorMessage << endl;
		status = 1;
	}
	catch (const Ice::Exception& ex) {
		std::cerr << tty::red << ex << tty::dcol << std::endl;
		status = 1;
	}
	catch (const char * msg) {
		std::cerr << msg << std::endl;
		status = 1;
	}
	if (ic) {
		ic->destroy();
	}
	return status;
}
