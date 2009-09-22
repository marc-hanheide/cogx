#include "TypeConversions.h"

#include <string>
#include <vector>
#include <cstdlib>

extern "C" {
#include "TypeConversions_mint.mh"
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

		for (int i = t->args.size() - 1; i >= 0; i--) {
			//margs = MR_list_cons(termToMercTerm(t->args[i], vs), margs);
			MR_term arg = termToMercTerm(t->args[i], vs);
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
	MR_atomic_formula maf = predicateToMercAtomicFormula(p->p, vs);
	MR_list__ctx_modality mm = modalitySeqToMercListOfModalities(p->m);

	MR_mprop__ctx_modality mprop;
	new_mprop(mm, maf, &mprop, *vs, vs);

	return mprop;
}

MR_with_cost_function__mprop__ctx_modality
withConstCostFunction(MR_mprop__ctx_modality mprop, double cost)
{
	MR_with_cost_function__mprop__ctx_modality result;
	new_with_const_cost_function(mprop, cost, &result);
	return result;
}

MR_ctx_modality
modalityToMercModality(const ModalityPtr & m)
{
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
			modality_k(&mm);
			break;

		default:
			cerr << "unsupported modality" << endl;
	}

	return mm;
}

MR_list__ctx_modality
modalitySeqToMercListOfModalities(const ModalitySeq & ms)
{
	MR_list__ctx_modality w_list;
	empty_ctx_modality_list(&w_list);

	for (int i = ms.size() - 1; i >= 0; i--) {
		MR_ctx_modality w_m = modalityToMercModality(ms[i]);
		cons_ctx_modality_list(w_m, w_list, &w_list);
	}

	return w_list;
}

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Agent
stringToAgent(const char * s)
{
	if (strcmp(s, "h") == 0) {
		//cerr << "human" << endl;
		return Human;
	}
	else if (strcmp(s, "r") == 0) {
		//cerr << "robot" << endl;
		return Robot;
	}
	else {
		cerr << "unknown agent in stringToAgent: \"" << s << "\"" << endl;
		return Robot;
	}
}

ModalityPtr
MR_WordToModality(MR_Word w)
{
//	cerr << "MR_WordToModality" << endl;
	MR_Word w_bel;

//	print_modality(w);

	if (is_modality_event(w)) {
		//cerr << "cc: event" << endl;
		ModalityPtr m = new Modality();
		m->type = Event;
		return m;
	}
	else if (is_modality_info(w)) {
		//cerr << "cc: info" << endl;
		ModalityPtr m = new Modality();
		m->type = Info;
		return m;
	}
	else if (is_modality_att(w)) {
		//cerr << "cc: att" << endl;
		ModalityPtr m = new Modality();
		m->type = AttState;
		return m;
	}
	else if (is_modality_k(w, &w_bel)) {
		//cerr << "cc: k" << endl;
		KModalityPtr km = new KModality();
		km->type = K;

		char * s1;
		char * s2;

		MR_Word w_strlist;

		if (is_belief_private(w_bel, &s1)) {
			//cerr << "private " << s1 << endl;
			km->share = Private;
			km->act = stringToAgent(s1);
		}
		else if (is_belief_attrib(w_bel, &s1, &s2)) {
			//cerr << "attrib " << s1 << " -> " << s2 << endl;
			km->share = Attribute;
			km->act = stringToAgent(s1);
			km->pat = stringToAgent(s2);
		}
		else if (is_belief_mutual(w_bel, &w_strlist)) {
			//cerr << "mutual" << endl;
			// XXX this!!
			km->share = Mutual;
			km->act = Human;
			km->act = Robot;
		}
		else {
			cerr << "unknown belief!" << endl;
			return 0;
		}
		return km;
	}
	else {
		cerr << "unknown modality!" << endl;
		return 0;
	}
}

ModalitySeq
MR_WordToModalitySeq(MR_Word w_list)
{
//	cerr << "MR_WordToModalitySeq" << endl;
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
//	cerr << "MR_WordToModalisedFormula" << endl;
	ModalisedFormulaPtr f = new ModalisedFormula();
	MR_Word w_m;
	MR_Word w_p;
	dissect_mprop(w_mf, &w_m, &w_p);
	f->m = MR_WordToModalitySeq(w_m);
	f->p = MR_WordToPredicate(w_vs, w_p);

	return f;
}

AbductiveProofPtr
MR_WordToAbductiveProof(MR_Word w_ctx, MR_Word w_proof)
{
//	cerr << "MR_WordToAbductiveProof" << endl;
	AbductiveProofPtr p = new AbductiveProof();
	
	MR_Word w_vs;
	MR_Word w_list;
	double cost;

	dissect_proof(w_ctx, w_proof, &w_vs, &w_list, &cost);

	MR_Word w_iter;
	for (w_iter = w_list; !MR_list_is_empty(w_iter); w_iter = MR_list_tail(w_iter)) {
		p->body.push_back(MR_WordToModalisedFormula(w_vs, MR_list_head(w_iter)));
	}

	p->cost = cost;

	return p;
}
