#include "common.h"

extern "C" {
#include "mercury_imp.h"
#include "MercuryAbducerServer_mint.mh"
}

#include <Ice/Ice.h>
#include <IceUtil/CtrlCHandler.h>
#include <Abducer.h>
#include <aserv.h>

#include <vector>

#include "TypeConversions.h"
#include "MercuryAbducerServer.h"
#include "ConsoleUtils.h"

using namespace std;

const char * SERVER_NAME = "AbducerServer";
const char * SERVER_ENDPOINTS = "default -p 10000";

// this probably shouldn't be static
static Ice::CommunicatorPtr ic;

void
shutdownServer(int);

int
aserv_main()
{
	IceUtil::CtrlCHandler ctrlCHandler(shutdownServer);
	int status = 0;
	try {
		ic = Ice::initialize();

		cerr << col::yel << "* server name = \"" << SERVER_NAME << "\"" << col::def << endl;
		cerr << col::yel << "* server endpoints = \"" << SERVER_ENDPOINTS << "\"" << col::def << endl;

		Ice::ObjectAdapterPtr adapter
				= ic->createObjectAdapterWithEndpoints("AbducerAdapter", SERVER_ENDPOINTS);

		Ice::ObjectPtr object = new MercuryAbducerServer();
		adapter->add(object, ic->stringToIdentity(SERVER_NAME));
		adapter->activate();

		ic->waitForShutdown();
	}
	catch (const Ice::Exception& e) {
		cerr << e << endl;
		status = 1;
	}
	catch (const char* msg) {
		cerr << msg << endl;
		status = 1;
	}

	cerr << col::yel << "* server shut down" << col::def << endl;

	if (ic) {
		try {
			ic->destroy();
		}
		catch (const Ice::Exception& e) {
			cerr << e << endl;
			status = 1;
		}
	}
	return status;
}

void
shutdownServer(int signum)
{
	cerr << endl;
	cerr << col::yel << "* received signal " << signum << col::def << endl;
	try {
		ic->shutdown();
	}
	catch (const Ice::Exception e) {
		cerr << e << endl;
		exit(1);
	}
}
