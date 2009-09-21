#ifndef TYPECONVERSIONS_H__
#define TYPECONVERSIONS_H__  1

extern "C" {
#include "mercury_imp.h"
}

#include <string>

#include "Abducer.h"

MR_Char *
stringToMercString(const std::string & s);

MR_Word
predicateToMercAtomicFormula(const Abducer::PredicatePtr & p, MR_Word * vs);

MR_Word
modalisedFormulaToMercMProp(const Abducer::ModalisedFormulaPtr & p, MR_Word * vs);

MR_Word
withConstCostFunction(MR_Word mprop, double cost);

MR_Word
modalityToMercModality(const Abducer::ModalityPtr & m);

//- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

Abducer::ModalityPtr
MR_WordToModality(MR_Word w);

Abducer::TermPtr
MR_WordToTerm(MR_Word w_vs, MR_Word w_t);

Abducer::PredicatePtr
MR_WordToPredicate(MR_Word w_vs, MR_Word w_p);

Abducer::ModalisedFormulaPtr
MR_WordToModalisedFormula(MR_Word w_vs, MR_Word w_mf);

Abducer::AbductiveProofPtr
MR_WordToAbductiveProof(MR_Word w_ctx, MR_Word w_proof);

#endif
