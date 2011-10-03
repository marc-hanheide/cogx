package de.dfki.lt.tr.infer.abducer.proof;

public abstract class AbstractProofInterpretationContext<T>
implements ProofInterpretationContext<T> {

	private final ProofPruner pruner;
	private final ProofExpander expander;
	private final AssertionSolver solver;
	private final ProofInterpreter interpreter;

	public AbstractProofInterpretationContext(ProofPruner pruner, ProofExpander expander, AssertionSolver solver, ProofInterpreter<T> interpreter) {
		this.pruner = pruner;
		this.expander = expander;
		this.solver = solver;
		this.interpreter = interpreter;
	}

	@Override
	public ProofPruner getPruner() {
		return pruner;
	}

	@Override
	public ProofExpander getExpander() {
		return expander;
	}

	@Override
	public AssertionSolver getSolver() {
		return solver;
	}

	@Override
	public ProofInterpreter<T> getInterpreter() {
		return interpreter;
	}

	@Override
	public abstract void onSuccessfulInterpretation(T interpretation);

	@Override
	public abstract void onNoInterpretation();
	
}
