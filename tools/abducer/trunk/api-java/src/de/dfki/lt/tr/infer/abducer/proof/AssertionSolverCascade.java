package de.dfki.lt.tr.infer.abducer.proof;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class AssertionSolverCascade
implements AssertionSolver {

	private final List<AssertionSolver> solvers;

	public AssertionSolverCascade() {
		this.solvers = new LinkedList<AssertionSolver>();
	}

	public void addSolver(AssertionSolver solver) {
		solvers.add(solver);
	}

	@Override
	public ContextUpdate solve(Assertion a) {
		ContextUpdate result = null;

		Iterator<AssertionSolver> iter = solvers.iterator();
		while (result == null && iter.hasNext()) {
			AssertionSolver solver = iter.next();
			if (solver != null) {
				result = solver.solve(a);
			}
		}
		
		return result;
	}

}
