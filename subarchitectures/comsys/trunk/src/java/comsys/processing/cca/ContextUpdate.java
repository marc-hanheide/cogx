package comsys.processing.cca;

import java.util.*;

import comsys.processing.cca.abduction.ProofUtils;

import Abducer.AssumedQuery;
import Abducer.IntentionModality;
import Abducer.MarkedQuery;
import Abducer.ModalisedFormula;
import Abducer.Predicate;
import beliefmodels.adl.*;
import beliefmodels.domainmodel.cogx.*;
import binder.utils.BeliefModelUtils;

public class ContextUpdate {

	private static Counter counter = new Counter("cca");
	
	public Predicate intention;
	//public MarkedQuery[] proof;
	public Belief[] beliefs;
	
	public ContextUpdate() {
		//this.proof = new MarkedQuery[] { };
		this.intention = null;
		this.beliefs = new Belief[] { };
	}
	
	public ContextUpdate(MarkedQuery[] proof) {
		//this.proof = proof;
		this.intention = proofToIntention(proof);
		this.beliefs = proofToBeliefs(proof);
	}
	
	public static Predicate proofToIntention(MarkedQuery[] proof) {
		ModalisedFormula[] assumptions = ProofUtils.proofToFacts(ProofUtils.filterAssumed(proof));

		// filter out just the event modality
		ArrayList<ModalisedFormula> list = new ArrayList<ModalisedFormula>();
		for (int i = 0; i < assumptions.length; i++) {
			if (assumptions[i].m.length == 1 && assumptions[i].m[0] instanceof IntentionModality) {
				list.add(assumptions[i]);
			}
		}
		ModalisedFormula[] fs = list.toArray(new ModalisedFormula[] { });
		if (fs.length == 1) {
			return fs[0].p;
		}
		else {
			return null;
		}
	}
	
	public static Belief[] proofToBeliefs(MarkedQuery[] proof) {
		Belief[] asserted = ProofUtils.extractAssertedBeliefs(proof);
		Belief[] assumed = ProofUtils.extractAssumedBeliefs(proof);
		Belief[] bs = new Belief[asserted.length + assumed.length];
		for (int i = 0; i < asserted.length; i++) {
			bs[i] = asserted[i];
			bs[i].id = counter.inc("cu-assert");
		}
		for (int j = 0; j < assumed.length; j++) {
			bs[j+asserted.length] = assumed[j];
			bs[j+asserted.length].id = counter.inc("cu-assume");
		}
		return bs;
	}
	
	public static Belief[] filterAssertedBeliefs(Belief[] bs) {
		Vector<Belief> tmp = new Vector<Belief>();
		for (int i = 0; i < bs.length; i++) {
			if (beliefHasAssertions(bs[i])) {
				tmp.add(bs[i]);
			}
		}
		return tmp.toArray(new Belief[]{});
	}

	public static boolean beliefHasAssertions(Belief b) {
		return formulaHasAssertions(b.phi);
	}

	private static boolean formulaHasAssertions(Formula f) {
		if (f instanceof ComplexFormula) {
			ComplexFormula cplxF = (ComplexFormula) f;
			for (int i = 0; i < cplxF.formulae.length; i++) {
				if (formulaHasAssertions(cplxF.formulae[i])) {
					return true;
				}
			}
		}
		if (f instanceof ContinualFormula) {
			ContinualFormula contF = (ContinualFormula) f;
			return contF.cstatus == ContinualStatus.assertion;
		}
		return false;
	}
	
	@Override
	public String toString() {
		String s = "[ContextUpdate\n";
		s += "  intention: " + MercuryUtils.predicateToString(intention) + "\n";
		s += "  beliefs:\n";
		for (int i = 0; i < beliefs.length; i++) {
			s += "  * " + BeliefModelUtils.getBeliefPrettyPrint(beliefs[i], 1);
		}
		s += "]\n";
		return s;
	}

}
