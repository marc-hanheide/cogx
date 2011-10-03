package de.dfki.lt.tr.infer.abducer.proof;

public interface ProofInterpretationContext<T> {

	public ProofPruner getPruner();

	public ProofExpander getExpander();

	public AssertionSolver getSolver();

	public ProofInterpreter<T> getInterpreter();

	public void onSuccessfulInterpretation(T interpretation);

	public void onNoInterpretation();

}
