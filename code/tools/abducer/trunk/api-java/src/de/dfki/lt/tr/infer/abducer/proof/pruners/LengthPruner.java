package de.dfki.lt.tr.infer.abducer.proof.pruners;

import de.dfki.lt.tr.infer.abducer.proof.Proof;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import java.util.Iterator;

public class LengthPruner
implements ProofPruner {

	private final int maxLength;

	public LengthPruner(int maxLength) {
		this.maxLength = maxLength;
	}

	@Override
	public void prune(Iterator<Proof> iter) {
		int i = 0;
		while (iter.hasNext()) {
			Proof p = iter.next();
			i++;

			if (i > maxLength) {
				iter.remove();
			}
		}
	}
	
}
