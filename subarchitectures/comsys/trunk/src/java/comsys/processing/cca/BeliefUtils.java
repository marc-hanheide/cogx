package comsys.processing.cca;

import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.ContinualFormula;
import beliefmodels.domainmodel.cogx.ContinualStatus;
import beliefmodels.domainmodel.cogx.SuperFormula;

public class BeliefUtils {

	public static SuperFormula changeAssertionsToPropositions(SuperFormula sf, float prob) {
		if (sf instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) sf;
			for (int i = 0; i < cf.formulae.length; i++) {
				cf.formulae[i] = changeAssertionsToPropositions(cf.formulae[i], prob);
			}
			return cf;
		}
		if (sf instanceof ContinualFormula) {
			ContinualFormula cof = (ContinualFormula) sf;
			if (cof.cstatus == ContinualStatus.assertion) {
				cof.cstatus = ContinualStatus.proposition;
				cof.prob = prob;
			}
			return cof;
		}
		return sf;
	}

	public static SuperFormula swapPolarityOfAssertions(SuperFormula sf) {
		if (sf instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) sf;
			for (int i = 0; i < cf.formulae.length; i++) {
				cf.formulae[i] = swapPolarityOfAssertions(cf.formulae[i]);
			}
			return cf;
		}
		if (sf instanceof ContinualFormula) {
			ContinualFormula cof = (ContinualFormula) sf;
			cof.polarity = !cof.polarity;
			return cof;
		}
		return sf;
	}

	
	public static boolean formulaHasAssertions(SuperFormula f) {
		if (f instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) f;
			for (int i = 0; i < cf.formulae.length; i++) {
				if (formulaHasAssertions(cf.formulae[i])) {
					return true;
				}
			}
			return false;
		}
		if (f instanceof ContinualFormula) {
			ContinualFormula cf = (ContinualFormula) f;
			return cf.cstatus == ContinualStatus.assertion;
		}
		return false;
	}

}
