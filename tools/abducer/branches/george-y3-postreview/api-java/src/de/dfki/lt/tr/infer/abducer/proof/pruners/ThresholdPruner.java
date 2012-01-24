package de.dfki.lt.tr.infer.abducer.proof.pruners;

import de.dfki.lt.tr.infer.abducer.proof.Proof;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import java.util.Iterator;
import java.util.NavigableSet;

public class ThresholdPruner
implements ProofPruner {

	private final double maxWeight;

	public ThresholdPruner(double maxWeight) {
		this.maxWeight = maxWeight;
	}

	@Override
	public void prune(NavigableSet<Proof> proofs) {
		Iterator<Proof> iter = proofs.iterator();
		while (iter.hasNext()) {
			Proof p = iter.next();
			if (p.getCost() > maxWeight) {
				iter.remove();
			}
		}
		
	}
	
}
