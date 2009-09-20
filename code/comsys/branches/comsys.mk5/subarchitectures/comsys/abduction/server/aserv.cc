#include "mercury_imp.h"
#include "iceserv_lib.mh"

#include <Ice/Ice.h>
#include <Abducer.h>
#include <aserv.h>

#include <vector>

using namespace std;
using namespace Abducer;

typedef MR_Word MercAbdCtx;

char *
stringToNewCharPtr(const string & s);

MR_Word
predicateToMercAtomicFormula(const PredicatePtr & p, MR_Word * vs);

MR_Word
modalisedFormulaToMercMProp(const ModalisedFormulaPtr & p, MR_Word * vs);

MR_Word
withConstCostFunction(MR_Word mprop, double cost);

MR_Word
modalityToMercModality(const ModalityPtr & m);

class AbducerServerI : public AbducerServer {
public:
	AbducerServerI();
	virtual void clearRules(const Ice::Current&);
	virtual void loadRulesFromFile(const std::string& filename, const Ice::Current&);

	virtual void clearFacts(const Ice::Current&);
	virtual void loadFactsFromFile(const std::string& filename, const Ice::Current&);
	virtual void addFact(const ModalisedFormulaPtr & f, const Ice::Current&);

	virtual ProofResult prove(const vector<AssumableGoalPtr> & g, const Ice::Current&);
	virtual AbductiveProofPtr getBestProof(const Ice::Current&);
private:
	MercAbdCtx ctx;

	bool haveProof;
	MR_Word curBestProof;
};

AbducerServerI::AbducerServerI()
{
	cerr << "[log] initialising abduction context" << endl;
	ctx = init_ctx();

	haveProof = false;
}

void
AbducerServerI::clearRules(const Ice::Current&)
{
	cerr << "[log] clearing explicit rules" << endl;
	clear_rules(ctx, &ctx);
}

void
AbducerServerI::loadRulesFromFile(const string& filename, const Ice::Current&)
{
	cerr << "[log] adding explicit rules from: " << filename << endl;

	char * s = new char[filename.length() + 1];
	copy(filename.begin(), filename.end(), s);
	s[filename.length()] = '\0';

	load_rules_from_file(s, ctx, &ctx);

	delete s;
}

void
AbducerServerI::clearFacts(const Ice::Current&)
{
	cerr << "[log] clearing explicit facts" << endl;
	clear_facts(ctx, &ctx);
}

void
AbducerServerI::loadFactsFromFile(const string& filename, const Ice::Current&)
{
	cerr << "[log] adding explicit facts from: " << filename << endl;

	char * s = new char[filename.length() + 1];
	copy(filename.begin(), filename.end(), s);
	s[filename.length()] = '\0';

	load_facts_from_file(s, ctx, &ctx);

	delete s;
}

void
AbducerServerI::addFact(const ModalisedFormulaPtr & fact, const Ice::Current&)
{
	cerr << "[log] adding fact " << fact->p->predSym << endl;

	MR_Word vs;
	new_varset(&vs);

	MR_Word mprop = modalisedFormulaToMercMProp(fact, &vs);
	add_mprop_fact(mprop, ctx, &ctx);
}

ProofResult
AbducerServerI::prove(const vector<AssumableGoalPtr> & goals, const Ice::Current&)
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

/*
	char * s = new char[g.length() + 1];
	copy(g.begin(), g.end(), s);
	s[g.length()] = '\0';

	//MR_Word bestProof;
	double proofCost;

	cerr << "[log] proving " << s << endl;

	if (prove_best(s, 10.0, ctx, &proofCost, &curBestProof)) {
		cerr << "  result: proof found" << endl;
		proof_summary(curBestProof, ctx);
		haveProof = true;
		return (SUCCESS);
	}
	else {
		cerr << "  result: no proof found" << endl;
		haveProof = false;
		return (FAILED);
	}

	delete s;
*/

/*
	cerr << "proving" << endl;

	cerr << "  predsym: " << g->body->p->predSym << endl;

	vector<ArgumentPtr>::iterator i = g->body->p->args.begin();
	for (i = g->body->p->args.begin(); i != g->body->p->args.end(); ++i) {
		cerr << "  arg: " << endl;

		ArgumentPtr & arg = *i;
		TermPtr & targ = 0;
		VarPtr & varg = 0;

		TermPtr targ = dynamic_cast<TermPtr &>(arg);
		VarPtr varg = dynamic_cast<VarPtr &>(arg);

	}

	return (ERROR);
*/

/*
	char * s = new char[g->body->termString.length() + 1];
	copy(g->body->termString.begin(), g->body->termString.end(), s);
	s[g->body->termString.length()] = '\0';

	MR_Word bestProof;
	double proofCost;

	if (prove_best(s, g->assumeCost, ctx, &proofCost, &bestProof)) {
		haveProof = true;
		return (SUCCESS);
	}
	else {
		haveProof = false;
		return (FAILED);
	}

	delete s;
*/
}


AbductiveProofPtr
AbducerServerI::getBestProof(const Ice::Current&)
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

char *
stringToNewCharPtr(const string & s)
{
	char * cs = new char[s.length() + 1];
	copy(s.begin(), s.end(), cs);
	cs[s.length()] = '\0';

	return cs;
}

MR_Word
termToMercTerm(const TermPtr & t, MR_Word * vs)
{
	if (t->variable) {
		char * s = stringToNewCharPtr(t->name);
		MR_Word mv;
		new_var(s, &mv, *vs, vs);
//		delete s;

		return mv;
	}
	else {
		MR_Word mt;
//		MR_Word margs = MR_list_empty();
		MR_Word margs;
		empty_term_list(&margs);

		for (int i = t->args.size() - 1; i >= 0; i--) {
			//margs = MR_list_cons(termToMercTerm(t->args[i], vs), margs);
			MR_Word arg = termToMercTerm(t->args[i], vs);
			cons_term_list(arg, margs, &margs);
		}

		char * s = stringToNewCharPtr(t->name);
		new_term(s, margs, &mt, *vs, vs);
//		delete s;

		return mt;
	}
}

MR_Word
predicateToMercAtomicFormula(const PredicatePtr & p, MR_Word * vs)
{
	MR_Word mp;
	MR_Word margs;
	empty_term_list(&margs);

	for (int i = p->args.size() - 1; i >= 0; i--) {
		MR_Word arg = termToMercTerm(p->args[i], vs);
		cons_term_list(arg, margs, &margs);
//		margs = MR_list_cons(arg, margs);
	}

	char * s = stringToNewCharPtr(p->predSym);
	new_atomic_formula(s, margs, &mp, *vs, vs);
//	delete s;

	return mp;
}

MR_Word
modalisedFormulaToMercMProp(const ModalisedFormulaPtr & p, MR_Word * vs)
{
	MR_Word maf = predicateToMercAtomicFormula(p->p, vs);
	MR_Word mm = modalityToMercModality(p->m);

	MR_Word mprop;
	new_mprop(mm, maf, &mprop, *vs, vs);

	return mprop;
}

MR_Word
withConstCostFunction(MR_Word mprop, double cost)
{
	MR_Word result;
	new_with_const_cost_function(mprop, cost, &result);
	return result;
}

MR_Word
modalityToMercModality(const ModalityPtr & m)
{
	MR_Word mm = 0;

	switch (m->type) {
		case Event: modality_event(&mm); break;
		case Info: modality_info(&mm); break;
		case AttState: modality_att(&mm); break;

		default:
			cerr << "unsupported modality" << endl;
	}
	return mm;
}
