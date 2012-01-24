package de.dfki.lt.tr.infer.abducer.proof;

public abstract class AbstractAssertionSolver<T> implements AssertionSolver {

	private ModalisedAtomMatcher<T> matcher;

	public AbstractAssertionSolver(ModalisedAtomMatcher<T> matcher) {
		this.matcher = matcher;
	}

	public abstract ContextUpdate solveFromParsed(T a);

	@Override
	public final ContextUpdate solve(Assertion a) {
		T parsedAssertion = matcher.match(a.getModalisedAtom());
		if (parsedAssertion == null) {
			return null;
		}
		else {
			return solveFromParsed(parsedAssertion);
		}
	}
	
}
