package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import java.util.ArrayList;
import java.util.List;

public class EngineProofExpander
implements ProofExpander {

	private final AbductionEnginePrx engine;
	private final int timeout;
	
	public EngineProofExpander(AbductionEnginePrx engine, int timeout) {
		if (engine == null) {
			throw new NullPointerException("engine is null");
		}
		this.engine = engine;
		this.timeout = timeout;
	}

	@Override
	public List<Proof> expand(Proof proof) {
		List<Proof> expansion = new ArrayList<Proof>();

		engine.startProving(proof.getMarkedQueries());
		List<ProofWithCost> result = engine.getProofs(timeout);

		for (ProofWithCost pwc : result) {
			expansion.add(new Proof(pwc));
		}

		return expansion;
	}

}
