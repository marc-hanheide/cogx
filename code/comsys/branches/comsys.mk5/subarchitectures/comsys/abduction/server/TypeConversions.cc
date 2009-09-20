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
