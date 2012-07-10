package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import java.util.Comparator;

class ProofWithCostComparator
implements Comparator<ProofWithCost> {

	public ProofWithCostComparator() {
	}

	public int compare(ProofWithCost o1, ProofWithCost o2) {
		return o1.cost < o2.cost ? -1 : (o1.cost == o2.cost ? 0 : +1);
	}

}