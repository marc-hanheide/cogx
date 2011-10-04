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

extern "C" {
#include <unistd.h>
}

#ifndef PROGRAM_NAME
#define PROGRAM_NAME  "??"
#endif

#include <Ice/Ice.h>
#include <IceUtil/CtrlCHandler.h>

#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

#include <iostream>

#include "asr-loquendo.h"

#include "Version.h"

#include "CLI.h"
#include "LoquendoRecogniserServer.h"

using namespace std;
using namespace log4cxx;
using namespace log4cxx::xml;

LoggerPtr mainLogger(Logger::getLogger("loquendo-asr.main"));

const string DEFAULT_SERVER_NAME = "LoquendoASRServer";
const string DEFAULT_SERVER_ENDPOINTS = "default -p 9021";
const string DEFAULT_ADAPTER_NAME = "LoquendoASRServerAdapter";

// this probably shouldn't be static, but we need it in order to
// handle Ctrl-C correctly
static Ice::CommunicatorPtr ic;
static LoquendoRecogniserServer * serverObject;

//------------------------------------------------------------------------------

void
printUsage();

void
printVersion();

int
runServer(Settings & setup);

void
shutdownServer(int);

//------------------------------------------------------------------------------

int
main(int argc, char ** argv)
{
	Settings setup;
	setup.serverName = DEFAULT_SERVER_NAME;
	setup.serverEndpoints = DEFAULT_SERVER_ENDPOINTS;
	setup.audiodumpDir = "";
	setup.exportWordLattices = false;

	switch (processCommandLineArgs(argc, argv, setup)) {
	case PrintHelp:
		printUsage();
		return EXIT_SUCCESS;

	case PrintVersion:
		printVersion();
		return EXIT_SUCCESS;

	case Start:
		{
			bool ok = true;
			if (setup.sessionFile == "") {
				cerr << PROGRAM_NAME << ": error: session file not specified" << endl;
				ok = false;
			}
			if (setup.grammarFile == "") {
				cerr << PROGRAM_NAME << ": error: grammar file not specified" << endl;
				ok = false;
			}
			if (!ok) {
				cerr << PROGRAM_NAME << ": type `" << PROGRAM_NAME << " -h' for help." << endl;
				ok = false;
				return EXIT_FAILURE;
			}
			printVersion();
			DOMConfigurator::configure("Log4jConfig.xml");
		}
		return runServer(setup);

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
	cout << "Usage: " << PROGRAM_NAME << " [OPTIONS] -s SESSION-FILE -g GRAMMAR-FILE" << endl
		<< "       " << PROGRAM_NAME << " -h" << endl
		<< "       " << PROGRAM_NAME << " -v" << endl
		<< endl
		<< "OPTIONS may be one or more of the following:" << endl
		<< "  -n NAME, --name NAME        ... set server name to NAME" << endl
		<< "                                  (default: `" << DEFAULT_SERVER_NAME << "')" << endl
		<< "  -e SPEC, --endpoints SPEC   ... set server endpoints to SPEC" << endl
		<< "                                  (default: `" << DEFAULT_SERVER_ENDPOINTS << "')" << endl
		<< "  -w, --word-lattices         ... export word lattices instead of n-best lists" << endl
		<< "  -d DIR                      ... dump audio files in the directory DIR" << endl
		<< "                                  (default: don't dump anything anywhere)" << endl;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
printVersion()
{
	cout << "Loquendo ASR server " << LOQICE_VERSION << endl;
	cout << "(c) 2010-2011 DFKI GmbH Talking Robots" << endl;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int
runServer(Settings & setup)
{
	IceUtil::CtrlCHandler ctrlCHandler(shutdownServer);
	int status = 0;
	try {
		ic = Ice::initialize();

		LOG4CXX_INFO(mainLogger, "setting up ICE server at " << setup.serverName << ":" << setup.serverEndpoints);

		Ice::ObjectAdapterPtr adapter
				= ic->createObjectAdapterWithEndpoints(DEFAULT_ADAPTER_NAME, setup.serverEndpoints);


		bool doAudiodumps = false;
		string audiodumpPrefix = "";
		string audiodumpSuffix = ".raw";

		if (setup.audiodumpDir != "") {
			doAudiodumps = true;
			audiodumpPrefix = setup.audiodumpDir + "/loqasr";
		}

		serverObject = new LoquendoRecogniserServer(ic, setup.sessionFile, setup.grammarFile, doAudiodumps, audiodumpPrefix, audiodumpSuffix, setup.exportWordLattices);
		adapter->add(serverObject, ic->stringToIdentity(setup.serverName));
		adapter->activate();

		ic->waitForShutdown();
	}
	catch (const InstanceException & e) {
		LOG4CXX_ERROR(mainLogger, "instance exception in function " << e.function << "(" << e.returnCode << "): " << e.message);
		status = 1;
	}
	catch (const SessionException & e) {
		LOG4CXX_ERROR(mainLogger, "session exception in function " << e.function << "(" << e.returnCode << "): " << e.message);
		status = 1;
	}
	catch (const Ice::Exception & e) {
		LOG4CXX_ERROR(mainLogger, "Ice exception: " << e);
		status = 1;
	}
	catch (const char* msg) {
		LOG4CXX_ERROR(mainLogger, "exception: " << msg);
		status = 1;
	}

	if (ic) {
		try {
			ic->destroy();
		}
		catch (const Ice::Exception& e) {
			cerr << e << endl;
			status = 1;
		}
	}

	LOG4CXX_INFO(mainLogger, "server shut down");
	return status;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void
shutdownServer(int signum)
{
	LOG4CXX_DEBUG(mainLogger, "received signal " << signum << ", shutting down the server");
	try {
//		LoquendoRecogniserServer * srv = static_cast<LoquendoRecogniserServer *>(&serverObject);
//		srv->stop();
//		serverObject->stop();
		serverObject->shutdown();
//		ic->destroy();
//		ic->shutdown();
	}
	catch (const Ice::Exception e) {
		cerr << e << endl;
		exit(1);
	}
}
