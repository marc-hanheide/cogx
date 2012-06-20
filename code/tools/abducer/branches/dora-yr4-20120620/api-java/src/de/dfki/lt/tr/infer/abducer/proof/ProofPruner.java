package de.dfki.lt.tr.infer.abducer.proof;

import java.util.NavigableSet;

public interface ProofPruner {

	void prune(NavigableSet<Proof> proofs);

}
