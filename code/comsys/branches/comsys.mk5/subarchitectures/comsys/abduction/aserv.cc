#include "mercury_imp.h"
#include "iceserv_lib.mh"

#include <Ice/Ice.h>
#include <Abducer.h>
#include <BeliefModels.h>
#include <aserv.h>

using namespace std;
using namespace autogen;
using namespace Abducer;

typedef MR_Word MercAbdCtx;

class AbducerServerI : public AbducerServer {
public:
	AbducerServerI();
	virtual void synchronise(const beliefmodels::adl::BeliefModelPtr& m, const Ice::Current&);
	virtual void clearRules(const Ice::Current&);
	virtual void loadRulesFromFile(const std::string& filename, const Ice::Current&);
	virtual ProofResult proveGoal(const goalSeq& g, const Ice::Current&);
	virtual AbductiveProofPtr getBestProof(const Ice::Current&);
private:
	MercAbdCtx ctx;

	bool haveProof;
	AbductiveProofPtr curBestProof;
};

AbducerServerI::AbducerServerI()
{
	cerr << "initialising abduction context" << endl;
	ctx = init_ctx();

	haveProof = false;
}

/*
void
AbducerServerI::clearFacts(const Ice::Current&)
{
	cerr << "clearing facts" << endl;

	MercAbdCtx newCtx;
	clear_facts(ctx, &newCtx);  // det
	ctx = newCtx;
}
*/

void
AbducerServerI::synchronise(const beliefmodels::adl::BeliefModelPtr& m, const Ice::Current&)
{
	cerr << "syncing" << endl;
}

void
AbducerServerI::clearRules(const Ice::Current&)
{
	cerr << "clearing rules" << endl;

	MercAbdCtx newCtx;
	clear_rules(ctx, &newCtx);  // det
	ctx = newCtx;
}

void
AbducerServerI::loadRulesFromFile(const string& filename,const Ice::Current&)
{
	cerr << "adding rules from: " << filename << endl;

	char * s = new char[filename.length() + 1];
	copy(filename.begin(), filename.end(), s);
	s[filename.length()] = '\0';

	load_rules_from_file(s, ctx, &ctx);

	delete s;
}

ProofResult
AbducerServerI::proveGoal(const goalSeq& g, const Ice::Current&)
{
//	cerr << "proving " << g->body->termString << endl;
	cerr << "proving" << endl;
	haveProof = false;
	return (ERROR);
}


AbductiveProofPtr
AbducerServerI::getBestProof(const Ice::Current&)
{
	// TODO: test that we have a proof in curBestProof
	return curBestProof;
}

int
aserv_main()
{
	int status = 0;
	Ice::CommunicatorPtr ic;
	try {
		ic = Ice::initialize();
		Ice::ObjectAdapterPtr adapter
				= ic->createObjectAdapterWithEndpoints("AbducerAdapter", "default -p 10000");
		Ice::ObjectPtr object = new AbducerServerI();
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
