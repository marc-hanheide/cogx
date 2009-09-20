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

#endif
