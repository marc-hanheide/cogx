#include "MercuryAbducerServer.h"

extern "C" {
#include "iceserv_lib.mh"
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
	cerr << "[log] adding explicit rules from: " << filename << endl;

	char * s = new char[filename.length() + 1];
	copy(filename.begin(), filename.end(), s);
	s[filename.length()] = '\0';

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
	cerr << "[log] adding explicit facts from: " << filename << endl;

	char * s = new char[filename.length() + 1];
	copy(filename.begin(), filename.end(), s);
	s[filename.length()] = '\0';

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
	add_mprop_fact(mprop, ctx, &ctx);
}

ProofResult
MercuryAbducerServer::prove(const vector<AssumableGoalPtr> & goals, const Ice::Current&)
{
	cerr << "[log] proving" << endl;

	MR_Word * vs;
	new_varset(vs);

	MR_Word mgs;
	empty_annots_list(&mgs);

	for (int i = 0; i < goals.size(); i++) {
		MR_Word mprop = modalisedFormulaToMercMProp((goals[i])->body, vs);
		MR_Word mannot = withConstCostFunction(mprop, (goals[i])->assumeCost);
		cons_annots_list(mannot, mgs, &mgs);
	}
	MR_Word minitproof;
	new_proof(mgs, *vs, &minitproof);

	double proofCost;

	if (prove_best(minitproof, ctx, &proofCost, &curBestProof)) {
		cerr << "  result: proof found" << endl;
		proof_summary(curBestProof, ctx);
		haveProof = true;
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
}

