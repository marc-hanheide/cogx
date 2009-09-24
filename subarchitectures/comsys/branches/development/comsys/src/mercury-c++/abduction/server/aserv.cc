#include "common.h"

extern "C" {
#include "mercury_imp.h"
#include "MercuryAbducerServer_mint.mh"
}

#include <Ice/Ice.h>
#include <Abducer.h>
#include <aserv.h>

#include <vector>

#include "TypeConversions.h"
#include "MercuryAbducerServer.h"

using namespace std;

int
aserv_main()
{
	int status = 0;
	Ice::CommunicatorPtr ic;
	try {
		ic = Ice::initialize();
		Ice::ObjectAdapterPtr adapter
				= ic->createObjectAdapterWithEndpoints("AbducerAdapter", "default -p 10000");
		Ice::ObjectPtr object = new MercuryAbducerServer();
		adapter->add(object, ic->stringToIdentity("AbducerServer"));
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
