#include "mercury_imp.h"
#include "iceserv_lib.mh"

#include <Ice/Ice.h>
#include <Abducer.h>
#include <aserv.h>

using namespace std;
using namespace autogen;
using namespace Abducer;

typedef MR_Word MercAbdCtx;

class AbducerServerI : public AbducerServer {
public:
	AbducerServerI();
	virtual void clearFacts(const Ice::Current&);
	virtual void clearRules(const Ice::Current&);
	virtual void addFact(const ModalisedFormulaPtr& fact, const Ice::Current&);
	virtual void addRule(const ModalisedFormulaPtr& rule, const Ice::Current&);
	virtual ProofResult proveGoal(const GoalPtr& g, const Ice::Current&);
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

void
AbducerServerI::clearFacts(const Ice::Current&)
{
	cerr << "clearing facts" << endl;

	MercAbdCtx newCtx;
	clear_facts(ctx, &newCtx);  // det
	ctx = newCtx;
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
AbducerServerI::addFact(const ModalisedFormulaPtr& fact,const Ice::Current&)
{
	cerr << "adding fact: " << fact->termString << endl;

	MercAbdCtx newCtx;
	char * s = new char[fact->termString.length() + 1];
	copy(fact->termString.begin(), fact->termString.end(), s);
	s[fact->termString.length()] = '\0';

	if (add_fact(s, ctx, &newCtx)) {
		// succeeded
		cerr << "succeeded" << endl;
		ctx = newCtx;
	}
	else {
		// failed
		cerr << "failed!" << endl;
	}

	delete s;
}

void
AbducerServerI::addRule(const ModalisedFormulaPtr& rule,const Ice::Current&)
{
	cerr << "adding rule: " << rule->termString << endl;

	MercAbdCtx newCtx;
	char * s = new char[rule->termString.length() + 1];
	copy(rule->termString.begin(), rule->termString.end(), s);
	s[rule->termString.length()] = '\0';

	if (add_rule(s, ctx, &newCtx)) {
		// succeeded
		cerr << "succeeded" << endl;
		ctx = newCtx;
	}
	else {
		// failed
		cerr << "failed!" << endl;
	}

	delete s;
}

ProofResult
AbducerServerI::proveGoal(const GoalPtr& g, const Ice::Current&)
{
	cerr << "proving " << g->body->termString << endl;
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
