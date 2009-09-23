#include "MercuryAbducerServer.h"

extern "C" {
#include "MercuryAbducerServer_mint.mh"
#include "TypeConversions_mint.mh"
}

#include "TypeConversions.h"

using namespace std;
using namespace Abducer;

#include <vector>

MercuryAbducerServer::MercuryAbducerServer()
{
	cerr << "[log] initialising abduction context" << endl;
	ctx = init_ctx();

	haveProof = false;
}

void
MercuryAbducerServer::clearRules(const Ice::Current&)
{
	cerr << "[log] clearing explicit rules" << endl;
	clear_rules(ctx, &ctx);
}

void
MercuryAbducerServer::loadRulesFromFile(const string& filename, const Ice::Current&)
{
	char * s = stringToMercString(filename);
	cerr << "[log] adding explicit rules from: " << s << endl;
	load_rules_from_file(s, ctx, &ctx);
	delete s;
}

void
MercuryAbducerServer::clearFacts(const Ice::Current&)
{
	cerr << "[log] clearing explicit facts" << endl;
	clear_facts(ctx, &ctx);
}

void
MercuryAbducerServer::loadFactsFromFile(const string& filename, const Ice::Current&)
{
	char * s = stringToMercString(filename);
	cerr << "[log] adding explicit facts from: " << filename << endl;
	load_facts_from_file(s, ctx, &ctx);
	delete s;
}

void
MercuryAbducerServer::addFact(const ModalisedFormulaPtr & fact, const Ice::Current&)
{
	cerr << "[log] adding fact " << fact->p->predSym << endl;

	MR_Word vs;
	new_varset(&vs);

	MR_Word mprop = modalisedFormulaToMercMProp(fact, &vs);
	add_mprop_fact(vs, mprop, ctx, &ctx);
}

void
MercuryAbducerServer::clearAssumables(const Ice::Current&)
{
	cerr << "[log] clearing assumables" << endl;
	clear_assumables(ctx, &ctx);
}

void
MercuryAbducerServer::addAssumable(const string & function, const ModalisedFormulaPtr & f, float cost, const Ice::Current&)
{
	cerr << "[log] adding assumable " << f->p->predSym << endl;

	MR_Word w_vs;
	new_varset(&w_vs);

	MR_Word w_mprop = modalisedFormulaToMercMProp(f, &w_vs);
	add_assumable(stringToMercString(function), w_mprop, cost, ctx, &ctx);	
}

ProveResult
MercuryAbducerServer::prove(const vector<MarkedQueryPtr> & goals, const Ice::Current&)
{
	cerr << "[log] proving" << endl;

	MR_Word * vs;
	new_varset(vs);

	MR_Word mgs;
	empty_marked_query_list(&mgs);

	for (int i = goals.size() - 1; i >= 0; i--) {
		MR_Word w_mq = markedQueryToMercQuery(goals[i], vs);
		cons_marked_query_list(w_mq, mgs, &mgs);
	}
	MR_Word minitproof;
	new_proof(mgs, *vs, &minitproof);

	double proofCost;

	if (prove_best(minitproof, ctx, &proofCost, &curBestProof)) {
		cerr << "  result: proof found" << endl;
		proof_summary(curBestProof, ctx);
		haveProof = true;
		cerr << " we're still alive!" << endl;
		return (SUCCESS);
	}
	else {
		cerr << "  result: no proof found" << endl;
		print_ctx(ctx);
		haveProof = false;
		return (FAILED);
	}
}

AbductiveProofPtr
MercuryAbducerServer::getBestProof(const Ice::Current&)
{
	cerr << "[log] requested the best proof" << endl;

	// TODO: test that we have a proof in curBestProof

	return MR_WordToAbductiveProof(ctx, curBestProof);
/*
	double cost;
	MR_Word assumed;
	MR_Word asserted;
	dissect_proof(curBestProof, ctx, &cost, &assumed, &asserted);

	MR_Word cur;
	vector<string> asmVect;
	vector<string> asrVect;

	for (cur = assumed; !MR_list_is_empty(cur); cur = MR_list_tail(cur)) {
		asmVect.push_back((const char *) MR_list_head(cur));
	}

	for (cur = asserted; !MR_list_is_empty(cur); cur = MR_list_tail(cur)) {
		asrVect.push_back((const char *) MR_list_head(cur));
	}

	AbductiveProofPtr proof = new AbductiveProof();
	proof->cost = cost;
	proof->assumed = asmVect;
	proof->asserted = asrVect;

	return proof;
*/
}

