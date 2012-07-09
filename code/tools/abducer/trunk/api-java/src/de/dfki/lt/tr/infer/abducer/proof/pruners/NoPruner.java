package de.dfki.lt.tr.infer.abducer.proof.pruners;

import de.dfki.lt.tr.infer.abducer.proof.Proof;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import java.util.Iterator;

public class NoPruner
implements ProofPruner {

	@Override
	public void prune(Iterator<Proof> proofs) {
		// intentionally left empty
	}

}
