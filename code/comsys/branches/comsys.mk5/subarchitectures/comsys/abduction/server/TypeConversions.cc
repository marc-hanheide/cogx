#include "TypeConversions.h"

#include <string>
#include <vector>

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

MR_Word
termToMercTerm(const TermPtr & t, MR_Word * vs)
{
	if (t->variable) {
		char * s = stringToMercString(t->name);
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

		char * s = stringToMercString(t->name);
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

	char * s = stringToMercString(p->predSym);
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

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

ModalityPtr
MR_WordToModality(MR_Word w)
{
	ModalityPtr m = new Modality();
	m->type = Event;
	return m;
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
	ModalisedFormulaPtr f = new ModalisedFormula();
	MR_Word w_m;
	MR_Word w_p;
	dissect_mprop(w_mf, &w_m, &w_p);
	f->m = MR_WordToModality(w_m);
	f->p = MR_WordToPredicate(w_vs, w_p);

	return f;
}

AbductiveProofPtr
MR_WordToAbductiveProof(MR_Word w_ctx, MR_Word w_proof)
{
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
