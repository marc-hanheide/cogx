package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class InterBeliefPointer {

	public String featName;
	public dBelief from;
	public dBelief to;

	public InterBeliefPointer(String featName_, dBelief from_, dBelief to_) {
		featName = featName_;
		from = from_;
		to = to_;
	}
}
