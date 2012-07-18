package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.beliefs.slice.distribs.*;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class CogXReferringUtils {

        public static boolean canBeReferredTo(dBelief bel) {
            if ("visualobject".equals(bel.type)) {
                if (bel.content instanceof CondIndependentDistribs) {
                    CondIndependentDistribs d = (CondIndependentDistribs) bel.content;
                    ProbDistribution pd = d.distribs.get("presence");
                    if (pd != null && pd instanceof BasicProbDistribution) {
                        BasicProbDistribution presd = (BasicProbDistribution) pd;
                        if (presd.values instanceof FormulaValues) {
                            FormulaValues fvs = (FormulaValues) presd.values;
                            if (fvs.values.size() == 1) {
                                FormulaProbPair fpp = fvs.values.get(0);
                                if (fpp.val instanceof ElementaryFormula) {
                                    ElementaryFormula formula = (ElementaryFormula) fpp.val;
                                    return "visible".equals(formula.prop);
                                }
                            }
                        }
                    }
                }
            }
            return false;
        }
    
}
