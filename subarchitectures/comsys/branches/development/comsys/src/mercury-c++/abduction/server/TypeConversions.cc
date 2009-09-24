#include "common.h"
#include "TypeConversions.h"

#include <string>
#include <vector>
#include <cstdlib>

extern "C" {
#include "TypeConversions_mint.mh"
#include <unistd.h>
}

using namespace std;
using namespace Abducer;

MR_Char *
stringToMercString(const string & s)
{
	char * cs = new char[s.length() + 1];
	memset(cs, '\0', s.length() + 1);
	s.copy(cs, s.length());

	return cs;
}

MR_term
termToMercTerm(const TermPtr & t, MR_varset * vs)
{
	debug(cerr << "termToMercTerm" << endl);
	if (t->variable) {
		char * s = stringToMercString(t->name);
		MR_term mv;
		new_var(s, &mv, *vs, vs);
//		delete s;

		return mv;
	}
	else {
		MR_term mt;
//		MR_Word margs = MR_list_empty();
		MR_list__term margs;
		empty_term_list(&margs);

//		vector<TermPtr>::const_reverse_iterator rit;
//		for (rit = t->args.rbegin(); rit != t->args.rend(); ++rit) {
		for (int i = t->args.size() - 1; i >= 0; i--) {
			//margs = MR_list_cons(termToMercTerm(t->args[i], vs), margs);
			MR_term arg = termToMercTerm(t->args[i], vs);
			//MR_term arg = termToMercTerm(*rit, vs);
			cons_term_list(arg, margs, &margs);
		}

		char * s = stringToMercString(t->name);
		new_term(s, margs, &mt, *vs, vs);
//		delete s;

		return mt;
	}
}

MR_atomic_formula
predicateToMercAtomicFormula(const PredicatePtr & p, MR_varset * vs)
{
	debug(cerr << "predicateToMercAtomicFormula" << endl);
	MR_atomic_formula mp;
	MR_list__term margs;
	empty_term_list(&margs);

	for (int i = p->args.size() - 1; i >= 0; i--) {
		MR_term arg = termToMercTerm(p->args[i], vs);
		cons_term_list(arg, margs, &margs);
//		margs = MR_list_cons(arg, margs);
	}

	char * s = stringToMercString(p->predSym);
	new_atomic_formula(s, margs, &mp, *vs, vs);
//	delete s;

	return mp;
}

MR_mprop__ctx_modality
modalisedFormulaToMercMProp(const ModalisedFormulaPtr & p, MR_varset * vs)
{
	debug(cerr << "modalisedFormulaToMercProp" << endl);
	MR_atomic_formula maf = predicateToMercAtomicFormula(p->p, vs);
	MR_list__ctx_modality mm = modalitySeqToMercListOfModalities(p->m);

	MR_mprop__ctx_modality mprop;
	new_mprop(mm, maf, &mprop, *vs, vs);

	return mprop;
}

MR_with_cost_function__mprop__ctx_modality
withConstCostFunction(MR_mprop__ctx_modality mprop, double cost)
{
	debug(cerr << "withConstCostFunction" << endl);
	MR_with_cost_function__mprop__ctx_modality result;
	new_with_const_cost_function(mprop, cost, &result);
	return result;
}

MR_ctx_modality
modalityToMercModality(const ModalityPtr & m)
{
	debug(cerr << "modalityToMercModality" << endl);
	MR_ctx_modality mm;

	switch (m->type) {
		case Event:
			modality_event(&mm);
			break;
		case Info:
			modality_info(&mm);
			break;
		case AttState:
			modality_att(&mm);
			break;
		case K:
			debug(cerr << "TODO: K modality!" << endl);
			modality_k(&mm);
			break;

		default:
			debug(cerr << "unsupported modality" << endl);
	}

	return mm;
}

MR_list__ctx_modality
modalitySeqToMercListOfModalities(const ModalitySeq & ms)
{
	debug(cerr << "modalitySeqToMercListOfModalities" << endl);
	MR_list__ctx_modality w_list;
	empty_ctx_modality_list(&w_list);

	vector<ModalityPtr>::const_reverse_iterator rit;
	debug(cerr << "  size = " << ms.size() << endl);
	for (rit = ms.rbegin(); rit != ms.rend(); ++rit) {
//	for (int i = ms.size() - 1; i >= 0; i--) {
		debug(cerr << "  trying" << endl);
		MR_ctx_modality w_m = modalityToMercModality(*rit);
		cons_ctx_modality_list(w_m, w_list, &w_list);
	}

	return w_list;
}

MR_Word
markedQueryToMercQuery(const MarkedQueryPtr & mq, MR_varset * w_vs)
{
	debug(cerr << "markedQueryToMercQuery" << endl);
	MR_Word w_mprop = modalisedFormulaToMercMProp(mq->body, w_vs);

	MR_Word w_query = 0;

	switch (mq->mark) {
	
	case Proved: {
			proved_query(w_mprop, &w_query);
		}
		break;

	case Unsolved: {
			UnsolvedQueryPtr uq = UnsolvedQueryPtr::dynamicCast(mq);
			MR_Word w_costfunc;
			if (uq->isConst) {
				const_cost_function(uq->constCost, &w_costfunc);
			}
			else {
				named_cost_function(stringToMercString(uq->costFunction), &w_costfunc);
			}
			unsolved_query(w_mprop, w_costfunc, &w_query);
		}
		break;
	
	case Assumed: {
			AssumedQueryPtr aq = AssumedQueryPtr::dynamicCast(mq);
			MR_Word w_costfunc;
			if (aq->isConst) {
				const_cost_function(aq->constCost, &w_costfunc);
			}
			else {
				named_cost_function(stringToMercString(aq->costFunction), &w_costfunc);
			}
			assumed_query(w_mprop, w_costfunc, &w_query);
		}
		break;
	
	case Asserted:
		{
			AssertedQueryPtr sq = AssertedQueryPtr::dynamicCast(mq);
			MR_Word w_list;
			empty_mprop_list(&w_list);

			for (int i = sq->antecedents.size() - 1; i >= 0; i--) {
				MR_ctx_modality w_m = modalisedFormulaToMercMProp(sq->antecedents[i], w_vs);
				cons_mprop_list(w_m, w_list, &w_list);
			}

			assumed_query(w_mprop, w_list, &w_query);
		}
		break;
	
	default:
		debug(cerr << "unknown marking in markedQueryToMercQuery!" << endl);
	}
	return w_query;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Agent
stringToAgent(const char * s)
{
	debug(cerr << "stringToAgent" << endl);
	if (strcmp(s, "h") == 0) {
		debug(cerr << "human" << endl);
		return Human;
	}
	else if (strcmp(s, "r") == 0) {
		debug(cerr << "robot" << endl);
		return Robot;
	}
	else {
		debug(cerr << "unknown agent in stringToAgent: \"" << s << "\"" << endl);
		return Robot;
	}
}

ModalityPtr
MR_WordToModality(MR_Word w)
{
	debug(cerr << "MR_WordToModality" << endl);
	MR_Word w_bel;

//	print_modality(w);

	if (is_modality_event(w)) {
		debug(cerr << "cc: event" << endl);
		ModalityPtr m = new Modality();
		m->type = Event;
		return m;
	}
	else if (is_modality_info(w)) {
		debug(cerr << "cc: info" << endl);
		ModalityPtr m = new Modality();
		m->type = Info;
		return m;
	}
	else if (is_modality_att(w)) {
		debug(cerr << "cc: att" << endl);
		ModalityPtr m = new Modality();
		m->type = AttState;
		return m;
	}
	else if (is_modality_k(w, &w_bel)) {
		debug(cerr << "cc: k" << endl);
		KModalityPtr km = new KModality();
		km->type = K;

		char * s1;
		char * s2;

		MR_Word w_strlist;

		if (is_belief_private(w_bel, &s1)) {
			debug(cerr << "private " << s1 << endl);
			km->share = Private;
			km->act = stringToAgent(s1);
			km->pat = Human; // so that it isn't uninitialised
		}
		else if (is_belief_attrib(w_bel, &s1, &s2)) {
			debug(cerr << "attrib " << s1 << " -> " << s2 << endl);
			km->share = Attribute;
			km->act = stringToAgent(s1);
			km->pat = stringToAgent(s2);
		}
		else if (is_belief_mutual(w_bel, &w_strlist)) {
			debug(cerr << "mutual" << endl);
			// XXX this!!
			km->share = Mutual;
			km->act = Human;
			km->act = Robot;
		}
		else {
			debug(cerr << "unknown belief!" << endl);
			return 0;
		}
		debug(cerr << "share = " << km->share << endl);
		debug(cerr << "act = " << km->act << endl);
		debug(cerr << "pat = " << km->pat << endl);
		return km;
	}
	else {
		debug(cerr << "unknown modality!" << endl);
		return 0;
	}
}

ModalitySeq
MR_WordToModalitySeq(MR_Word w_list)
{
	debug(cerr << "MR_WordToModalitySeq" << endl);
	ModalitySeq seq = vector<ModalityPtr>();

//	print_list_modalities(w_list);

	MR_Word w_iter;
	for (w_iter = w_list; !MR_list_is_empty(w_iter); w_iter = MR_list_tail(w_iter)) {
		seq.push_back(MR_WordToModality(MR_list_head(w_iter)));
	}
	return seq;
}

TermPtr
MR_WordToTerm(MR_Word w_vs, MR_Word w_t)
{
	debug(cerr << "MR_WordToTerm" << endl);
	char * name;
	MR_Bool is_var;
	MR_Word w_list;
	dissect_term(w_vs, w_t, (MR_Word*)&is_var, &name, &w_list);

	TermPtr t = new Term();
	t->variable = (is_var == MR_YES);
	t->name = name;
	t->args = vector<TermPtr>();

	if (t->variable) {
		MR_Word w_iter;
		for (w_iter = w_list; !MR_list_is_empty(w_iter); w_iter = MR_list_tail(w_iter)) {
			t->args.push_back(MR_WordToTerm(w_vs, MR_list_head(w_iter)));
		}
	}

	return t;
}

PredicatePtr
MR_WordToPredicate(MR_Word w_vs, MR_Word w_p)
{
	debug(cerr << "MR_WordToPredicate" << endl);
	char * predSym;
	MR_Word w_list;
	dissect_predicate(w_vs, w_p, &predSym, &w_list);

	PredicatePtr p = new Predicate();
	p->predSym = predSym;
	p->args = vector<TermPtr>();

	MR_Word w_iter;
	for (w_iter = w_list; !MR_list_is_empty(w_iter); w_iter = MR_list_tail(w_iter)) {
		p->args.push_back(MR_WordToTerm(w_vs, MR_list_head(w_iter)));
	}

	return p;
}

ModalisedFormulaPtr
MR_WordToModalisedFormula(MR_Word w_vs, MR_Word w_mf)
{
	debug(cerr << "MR_WordToModalisedFormula" << endl);
	ModalisedFormulaPtr f = new ModalisedFormula();
	MR_Word w_m;
	MR_Word w_p;
	dissect_mprop(w_mf, &w_m, &w_p);
	f->m = MR_WordToModalitySeq(w_m);
	f->p = MR_WordToPredicate(w_vs, w_p);

	return f;
}

MarkedQueryPtr
MR_WordToMarkedQuery(MR_Word w_vs, MR_Word w_mq)
{
	debug(cerr << "MR_WordToMarkedQuery" << endl);
	MR_Word w_arg1;
	MR_Word w_arg2;

	if (is_proved_query(w_mq, &w_arg1)) {
		debug(cerr << "  cc: is_proved_query" << endl);
		ProvedQueryPtr pq = new ProvedQuery();
		pq->mark = Proved;
		pq->body = MR_WordToModalisedFormula(w_vs, w_arg1);
		return pq;
	}
	else if (is_unsolved_query(w_mq, &w_arg1, &w_arg2)) {
		debug(cerr << "  cc: is_unsolved_query" << endl);
		UnsolvedQueryPtr uq = new UnsolvedQuery();
		uq->mark = Unsolved;
		uq->body = MR_WordToModalisedFormula(w_vs, w_arg1);
		uq->isConst = true;
		uq->constCost = 1.0;
		uq->costFunction = "";
		return uq;
	}
	else if (is_assumed_query(w_mq, &w_arg1, &w_arg2)) {
		debug(cerr << "  cc: is_assumed_query" << endl);
		AssumedQueryPtr asmq = new AssumedQuery();
		asmq->mark = Assumed;
		asmq->body = MR_WordToModalisedFormula(w_vs, w_arg1);
		asmq->isConst = true;
		asmq->constCost = 1.0;
		asmq->costFunction = "";
		return asmq;
	}
	else if (is_asserted_query(w_mq, &w_arg1, &w_arg2)) {
		debug(cerr << "  cc: is_asserted_query" << endl);
		AssertedQueryPtr asrq = new AssertedQuery();
		asrq->mark = Asserted;
		asrq->body = MR_WordToModalisedFormula(w_vs, w_arg1);
		asrq->antecedents = vector<ModalisedFormulaPtr>();
		MR_Word w_iter;
		for (w_iter = w_arg2; !MR_list_is_empty(w_iter); w_iter = MR_list_tail(w_iter)) {
			asrq->antecedents.push_back(MR_WordToModalisedFormula(w_vs, MR_list_head(w_iter)));
		}
		return asrq;
	}
	else {
		debug(cerr << "unknown marked query!" << endl);
		return 0;
	}
}

AbductiveProofPtr
MR_WordToAbductiveProof(MR_Word w_ctx, MR_Word w_proof)
{
	debug(cerr << "MR_WordToAbductiveProof" << endl);
	AbductiveProofPtr p = new AbductiveProof();
	p->body = vector<MarkedQueryPtr>();
	
	MR_Word w_vs;
	MR_Word w_list;
	double cost;

	dissect_proof(w_ctx, w_proof, &w_vs, &w_list, &cost);

	MR_Word w_iter;
	for (w_iter = w_list; !MR_list_is_empty(w_iter); w_iter = MR_list_tail(w_iter)) {
		p->body.push_back(MR_WordToMarkedQuery(w_vs, MR_list_head(w_iter)));
	}

	p->cost = cost;

	return p;
}
