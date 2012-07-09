package de.dfki.lt.tr.infer.abducer.proof;

import java.util.List;

public interface ProofExpander {

	public List<Proof> expand(Proof pwc);
	
}
