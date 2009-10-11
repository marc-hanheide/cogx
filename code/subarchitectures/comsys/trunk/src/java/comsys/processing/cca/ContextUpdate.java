package comsys.processing.cca;

import java.util.ArrayList;

import Abducer.AssumedQuery;
import Abducer.EventModality;
import Abducer.MarkedQuery;
import Abducer.ModalisedFormula;
import Abducer.Predicate;


public class ContextUpdate {

	public Predicate intention;
	public MarkedQuery[] proof;
	
	public ContextUpdate() {
		this.proof = new MarkedQuery[] { };
		this.intention = null;
	}
	
	public ContextUpdate(MarkedQuery[] proof) {
		this.proof = proof;
		this.intention = proofToIntention(proof);
	}
	
	public static Predicate proofToIntention(MarkedQuery[] proof) {
		ModalisedFormula[] assumptions = ProofUtils.proofToFacts(ProofUtils.filterAssumed(proof));

		// filter out just the event modality
		ArrayList<ModalisedFormula> list = new ArrayList<ModalisedFormula>();
		for (int i = 0; i < assumptions.length; i++) {
			if (assumptions[i].m.length == 1 && assumptions[i].m[0] instanceof EventModality) {
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
}
