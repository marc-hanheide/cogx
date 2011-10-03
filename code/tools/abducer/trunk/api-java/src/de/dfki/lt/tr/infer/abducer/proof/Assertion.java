package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.engine.AbductionEnginePrx;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.NullAssumabilityFunction;

public class Assertion {

	private final ProofSet proofSet;
	private final Proof p;
	private final int index;

	private final ModalisedAtom matom;

	Assertion(ProofSet proofSet, Proof p, int index) {
		this.proofSet = proofSet;
		this.p = p;

		this.index = index;
		
		matom = this.p.getMarkedQueries().get(this.index).atom;
	}

	private void ready() {
		MarkedQuery newQuery = new UnsolvedQuery(matom, new NullAssumabilityFunction());
		p.getMarkedQueries().set(index, newQuery);
	}

	private void failed() {
		// remove this proof from the proof set!
		proofSet.removeProof(p);
	}

	public ModalisedAtom getModalisedAtom() {
		return matom;
	}

	public ContextUpdate solve(AssertionSolver solver) {
		return solver.solve(this);
	}

	public boolean process(AssertionSolver solver, AbductionEnginePrx engine) {
		ContextUpdate update = solve(solver);
		if (update != null) {
			update.doUpdate(engine);
			ready();
			return true;
		}
		else {
			failed();
			return false;
		}
	}

}
