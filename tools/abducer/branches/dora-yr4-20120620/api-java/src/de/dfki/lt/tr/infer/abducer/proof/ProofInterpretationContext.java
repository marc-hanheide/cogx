package de.dfki.lt.tr.infer.abducer.proof;

import java.util.List;

public interface ProofInterpretationContext<T, U> {

	public ProofPruner getPruner();

	public ProofExpander getExpander();

	public AssertionSolver getSolver();

	public ProofInterpreter<T> getInterpreter();

	public void onSuccessfulInterpretation(List<T> interpretation, double priorConfidence, U arg);

	public void onNoInterpretation();

}
