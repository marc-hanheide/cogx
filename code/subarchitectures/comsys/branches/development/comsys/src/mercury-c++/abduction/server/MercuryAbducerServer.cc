#include "common.h"
#include "MercuryAbducerServer.h"

extern "C" {
#include "MercuryAbducerServer_mint.mh"
#include "TypeConversions_mint.mh"
#include <unistd.h>
}

#include "TypeConversions.h"
#include "ConsoleUtils.h"

using namespace std;
using namespace Abducer;

#include <vector>

MercuryAbducerServer::MercuryAbducerServer()
{
	cerr << col::grn << "* initialising abducer context" << col::def << endl;
	ctx = init_ctx();

	haveProof = false;
}

void
MercuryAbducerServer::clearRules(const Ice::Current&)
{
	cerr << col::grn << "* clearing rules" << col::def << endl;
	clear_rules(ctx, &ctx);
}

void
MercuryAbducerServer::loadRulesFromFile(const string& filename, const Ice::Current&)
{
	char * s = cc2m::string(filename);
	cerr << col::grn << "* loading rules from `" << s << "'" << col::def << endl;
	load_rules_from_file(s, ctx, &ctx);
	delete s;
}

void
MercuryAbducerServer::clearFacts(const Ice::Current&)
{
	cerr << col::grn << "* clearing facts" << col::def << endl;
	clear_facts(ctx, &ctx);
}

void
MercuryAbducerServer::loadFactsFromFile(const string& filename, const Ice::Current&)
{
	char * s = cc2m::string(filename);
	cerr << col::grn << "* loading facts from `" << s << "'" << col::def << endl;
	load_facts_from_file(s, ctx, &ctx);
	delete s;
}

void
MercuryAbducerServer::addFact(const ModalisedFormulaPtr & fact, const Ice::Current&)
{
	cerr << col::grn << "* adding fact: " << fact->p->predSym << "(...)" << col::def << endl;

	MR_Word vs;
	new_varset(&vs);

	MR_Word mprop = modalisedFormulaToMercMProp(fact, &vs);
	add_mprop_fact(vs, mprop, ctx, &ctx);
}

void
MercuryAbducerServer::clearAssumables(const Ice::Current&)
{
	cerr << col::grn << "* clearing assumables" << col::def << endl;
	clear_assumables(ctx, &ctx);
}

void
MercuryAbducerServer::addAssumable(const string & function, const ModalisedFormulaPtr & f, float cost, const Ice::Current&)
{
	cerr << col::grn << "* adding assumable: " << f->p->predSym << "(...) / " << function << col::def << endl;

	MR_Word w_vs;
	new_varset(&w_vs);

	MR_Word w_mprop = modalisedFormulaToMercMProp(f, &w_vs);
	add_assumable(cc2m::string(function), w_mprop, cost, ctx, &ctx);	
}

ProveResult
MercuryAbducerServer::prove(const vector<MarkedQueryPtr> & goals, const Ice::Current&)
{
	cerr << col::grn << "* proving" << col::def << endl;

	MR_Word vs;
	new_varset(&vs);

	MR_Word mgs;
	empty_marked_query_list(&mgs);

	debug(cerr << "  no of goals = " << goals.size() << endl);
	vector<MarkedQueryPtr>::const_reverse_iterator rit;
//	for (int i = goals.size() - 1; i >= 0; i--) {
	for (rit = goals.rbegin(); rit != goals.rend(); ++rit) {
		debug(cerr << "  doing a goal" << endl);
		MR_Word w_mq = markedQueryToMercQuery(*rit, &vs);
		//MR_Word w_mq = markedQueryToMercQuery(goals[i], vs);
		cons_marked_query_list(w_mq, mgs, &mgs);
	}
	MR_Word minitproof;
	new_proof(mgs, vs, &minitproof);

	double proofCost;

	if (prove_best(minitproof, ctx, &proofCost, &curBestProof)) {
		cerr << "proof found" << endl;
		cerr << endl;
		proof_summary(curBestProof, ctx);
		haveProof = true;
		//sleep(1);
		cerr << " we're still alive!" << endl;
		//sleep(1);
		return (ProofFound);
	}
	else {
		cerr << "no proof found" << endl;
		//print_ctx(ctx);
		haveProof = false;
		return (NoProofFound);
	}
}

vector<MarkedQueryPtr>
MercuryAbducerServer::getBestProof(const Ice::Current&)
{
	cerr << col::grn << "* sending the last proof" << col::def << endl;

	if (haveProof) {
		return MR_WordToMarkedQuerySeq(ctx, curBestProof);
	}
	else {
		cerr << "ERROR: no proof" << endl;
		throw NoProofException();
	}
}

