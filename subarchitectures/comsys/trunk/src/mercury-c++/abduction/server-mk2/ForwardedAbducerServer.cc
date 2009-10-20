#include "common.h"
#include "ForwardedAbducerServer.h"

#include "TtyUtils.h"

using namespace std;
using namespace Abducer;

#include <vector>

static const size_t bufsize = 8192;

static char buf[bufsize];

ForwardedAbducerServer::ForwardedAbducerServer()
{
	cerr << tty::green << "* initialising abducer context" << tty::dcol << endl;
//	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;

	cout << "init_ctx." << endl;
}

void
ForwardedAbducerServer::loadFile(const string& filename, const Ice::Current&)
{
	cerr << tty::green << "* loading file `" << filename << "'" << tty::dcol << endl;
//	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;

	cout << "load_file(\"" << filename << "\")." << endl;

	if (cin) {
		cin.getline(buf, bufsize);
		cerr << "load file reply: " << buf << endl;
	}

/*
	MR_Word w_result;
	char * args;
	int argi;

	load_file(s, &w_result, ctx, &ctx);

	if (load_result_is_ok(w_result)) {
		cout << tty::green << "  result: ok" << tty::dcol << endl;
	}
	else if (load_result_is_file_read_error(w_result)) {
		cout << tty::red << "  file read error" << tty::dcol << endl;
		throw FileReadErrorException(filename);
	}
	else if (load_result_is_syntax_error(w_result, &args, &argi)) {
		cout << tty::red << "  syntax error: " << args << " on line " << argi << tty::dcol << endl;
		throw SyntaxErrorException(filename, args, argi);
	}
*/
}

void
ForwardedAbducerServer::clearRules(const Ice::Current&)
{
	cerr << tty::green << "* clearing rules" << tty::dcol << endl;
//	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
	cout << "clear_rules." << endl;
}

void
ForwardedAbducerServer::clearFacts(const Ice::Current&)
{
	cerr << tty::green << "* clearing all facts" << tty::dcol << endl;
//	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
	cout << "clear_facts." << endl;
}

void
ForwardedAbducerServer::clearFactsByModality(ModalityType type, const Ice::Current&)
{
	switch (type) {
		case Event:
			cerr << tty::green << "* clearing Event facts" << tty::dcol << endl;
			cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
//			clear_e_facts(ctx, &ctx);
			break;

		case Info:
			cerr << tty::green << "* clearing Info facts" << tty::dcol << endl;
			cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
//			clear_i_facts(ctx, &ctx);
			break;

		case AttState:
			cerr << tty::green << "* clearing AttState facts" << tty::dcol << endl;
			cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
//			clear_a_facts(ctx, &ctx);
			break;

		case K:
			cerr << tty::green << "* clearing K facts" << tty::dcol << endl;
			cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
//			clear_k_facts(ctx, &ctx);
			break;

		default:
			cerr << tty::red << "* asked to clear facts with unknown modality!" << tty::dcol << endl;
			break;
	}
}

void
ForwardedAbducerServer::clearAssumables(const Ice::Current&)
{
	cerr << tty::green << "* clearing assumables" << tty::dcol << endl;
//	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;
	cout << "clear_assumables." << endl;
}

void
ForwardedAbducerServer::addFact(const ModalisedFormulaPtr & fact, const Ice::Current&)
{
	cerr << tty::green << "* adding fact: " << fact->p->predSym << "(...)" << tty::dcol << endl;
	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;

/*
	MR_Word vs;
	new_varset(&vs);

	MR_Word mprop = modalisedFormulaToMercMProp(fact, &vs);
	add_mprop_fact(vs, mprop, ctx, &ctx);
*/
}

void
ForwardedAbducerServer::addAssumable(const string & function, const ModalisedFormulaPtr & f, float cost, const Ice::Current&)
{
	cerr << tty::green << "* adding assumable: " << f->p->predSym << "(...) / " << function << tty::dcol << endl;
	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;

/*
	MR_Word w_vs;
	new_varset(&w_vs);

	MR_Word w_mprop = modalisedFormulaToMercMProp(f, &w_vs);
	add_assumable(cc2m::string(function), w_mprop, cost, ctx, &ctx);	
*/
}

ProveResult
ForwardedAbducerServer::prove(const vector<MarkedQueryPtr> & goals, const Ice::Current&)
{
	cerr << tty::green << "* proving" << tty::dcol << endl;
	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;

	return Error;
/*
	MR_Word vs;
	new_varset(&vs);

	MR_Word mgs;
	empty_marked_query_list(&mgs);

	debug(cout << "  no of goals = " << goals.size() << endl);
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

	print_ctx(ctx);
	cout << endl;

	if (prove_best(minitproof, ctx, &proofCost, &curBestProof)) {
		cout << "RESULT: proof found" << endl;
		cout << endl;
		proof_summary(curBestProof, ctx);
		haveProof = true;
		//sleep(1);
		debug(cerr << " we're still alive!" << endl);
		//sleep(1);
		return (ProofFound);
	}
	else {
		cout << "RESULT: no proof found" << endl;
		//print_ctx(ctx);
		haveProof = false;
		return (NoProofFound);
	}
*/
}

vector<MarkedQueryPtr>
ForwardedAbducerServer::getBestProof(const Ice::Current&)
{
	cerr << tty::green << "* retrieving the last proof" << tty::dcol << endl;
	cerr << tty::red << "  [unimplemented]" << tty::dcol << endl;

	throw NoProofException();

/*
	if (haveProof) {
		return MR_WordToMarkedQuerySeq(ctx, curBestProof);
	}
	else {
		cout << "ERROR: no proof" << endl;
		throw NoProofException();
	}
*/
}

