package de.dfki.lt.tr.infer.abducer.proof;

public interface AssertionSolver {

	public ContextUpdate solve(Assertion a);

}
