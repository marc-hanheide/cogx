package de.dfki.lt.tr.dialogue.epmodel.ice;

import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.dialogue.epmodel.CNF;
import de.dfki.lt.tr.dialogue.epmodel.IntentionIface;
import de.dfki.lt.tr.dialogue.epmodel.StateFormula;

public class IceIntention implements IntentionIface {

	private final Intention ice_intn;

	public IceIntention(Intention intn) {
		ice_intn = intn;
	}

	public static IceIntention fromIce(Intention intn) {
		return new IceIntention(intn);
	}

	public Intention toIce() {
		return ice_intn;
	}

	public CNF<StateFormula> getPreconditions() {
		throw new UnsupportedOperationException("Not supported yet.");
	}

	public CNF<StateFormula> getPostconditions() {
		throw new UnsupportedOperationException("Not supported yet.");
	}

}
