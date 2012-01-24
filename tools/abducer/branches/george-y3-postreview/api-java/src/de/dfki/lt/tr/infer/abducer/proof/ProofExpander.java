package de.dfki.lt.tr.infer.abducer.proof;

import java.util.Set;

public interface ProofExpander {

	public Set<Proof> expand(Proof pwc);
	
}
