package de.dfki.lt.tr.infer.abducer.proof;

import java.util.Iterator;

public interface ProofPruner {

	void prune(Iterator<Proof> proofs);

}
