package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;

public abstract class AbstractAssertionSolver<T> implements AssertionSolver {

	public abstract T parseFromModalisedAtom(ModalisedAtom matom);

	public abstract ContextUpdate solveFromParsed(T a);

	@Override
	public final ContextUpdate solve(Assertion a) {
		T parsedAssertion = parseFromModalisedAtom(a.getModalisedAtom());
		if (parsedAssertion == null) {
			return null;
		}
		else {
			return solveFromParsed(parsedAssertion);
		}
	}
	
}
