#ifndef TYPECONVERSIONS_H__
#define TYPECONVERSIONS_H__  1

extern "C" {
#include "mercury_imp.h"
}

#include <string>

#include "Abducer.h"

typedef MR_Word  MR_ctx;
typedef MR_Word  MR_varset;
typedef MR_Word  MR_term;
typedef MR_Word  MR_atomic_formula;
typedef MR_Word  MR_mprop__ctx_modality;
typedef MR_Word  MR_with_cost_function__mprop__ctx_modality;
typedef MR_Word  MR_ctx_modality;
typedef MR_Word  MR_proof__ctx_modality;

typedef MR_Word  MR_list__term;
typedef MR_Word  MR_list__ctx_modality;

MR_Char *
stringToMercString(const std::string & s);

MR_term
termToMercTerm(const Abducer::TermPtr & t, MR_varset * w_vs);

MR_atomic_formula
predicateToMercAtomicFormula(const Abducer::PredicatePtr & p, MR_varset * w_vs);

MR_mprop__ctx_modality
modalisedFormulaToMercMProp(const Abducer::ModalisedFormulaPtr & p, MR_varset * w_vs);

MR_with_cost_function__mprop__ctx_modality
withConstCostFunction(MR_mprop__ctx_modality w_mprop, double cost);

MR_ctx_modality
modalityToMercModality(const Abducer::ModalityPtr & m);

MR_list__ctx_modality
modalitySeqToMercListOfModalities(const Abducer::ModalitySeq & ms);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Abducer::ModalityPtr
MR_WordToModality(MR_ctx_modality w);

Abducer::TermPtr
MR_WordToTerm(MR_varset w_vs, MR_term w_t);

Abducer::PredicatePtr
MR_WordToPredicate(MR_varset w_vs, MR_atomic_formula w_p);

Abducer::ModalisedFormulaPtr
MR_WordToModalisedFormula(MR_varset w_vs, MR_mprop__ctx_modality w_mf);

Abducer::AbductiveProofPtr
MR_WordToAbductiveProof(MR_ctx w_ctx, MR_proof__ctx_modality w_proof);

#endif
