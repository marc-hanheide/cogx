package de.dfki.lt.tr.infer.abducer.proof;

import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.NoSuchElementException;
import java.util.Set;
import java.util.TreeSet;
import org.apache.log4j.Logger;

public class ProofSet {

	private final Logger logger;
	private final ProofPruner pruner;
	private final TreeSet<Proof> proofs;

	private ProofSet(Logger logger, ProofPruner pruningMethod, Collection<Proof> proofs) {
		this.logger = logger;
		this.proofs = new TreeSet<Proof>(proofs);
		this.pruner = pruningMethod;
	}

	public ProofSet(Logger logger, ProofPruner pruningMethod) {
		this(logger, pruningMethod, Collections.<Proof>emptySet());
	}

	public ProofSet(Logger logger, ProofPruner pruningMethod, Proof p) {
		this(logger, pruningMethod, Collections.singleton(p));
	}

	public Iterator<Proof> iterator() {
		return proofs.iterator();
	}

	public Proof getMostLikely() {
		try {
			return proofs.first();
		}
		catch (NoSuchElementException ex) {
			return null;
		}
	}

	public boolean isUnsolved() {
		for (Proof p : proofs) {
			if (p.isUnsolved()) {
				return true;
			}
		}
		return false;
	}

	public void removeProof(Proof p) {
		proofs.remove(p);
	}

	@Deprecated
	public void expandAll(ProofExpander expander) {
		Set<Proof> toAdd = new TreeSet<Proof>();
		for (Proof p : proofs) {
			toAdd.addAll(expander.expand(p));
		}
		proofs.clear();
		proofs.addAll(toAdd);
		prune();
	}

	public <T, U> ExpansionStepResult<T> expansionStep(ProofInterpretationContext<T, U> ctx) {
		logger.trace("BEGIN-expansion");
		Proof p = getMostLikely();
		if (p == null) {
			logger.trace("END-expansion: no proofs in the proof set");
			return ExpansionStepResult.failedResult();
		}

		logger.trace("this is the most likely proof:\n" + PrettyPrint.proofToString(p.getMarkedQueries()));

		if (p.isStable()) {
			logger.trace("proof is stable");
			T interpretation = ctx.getInterpreter().interpret(p.toModalisedAtoms(), p.getCost());
			if (interpretation == null) {
				// interpretation step failed
				logger.trace("interpretation returned null -> removing the proof (will continue)");
				proofs.remove(p);
				return expansionStep(ctx);
			}
			else {
				logger.trace("END-expansion: a non-null interpretation found");
				logger.trace("removing the proof");
				proofs.remove(p);
				return ExpansionStepResult.successResult(interpretation);
			}
		}
		else if (p.hasAssertions()) {
			logger.trace("the proof has assertions");
			Assertion a = p.getFirstAssertion(this);
			
			// register a listener for changes there?
			logger.trace("END-expansion: assertion slated for processing");
			return ExpansionStepResult.inProgressResult(a);
		}
		else {
			logger.trace("the proof has unsolved queries, will expand");
			Set<Proof> expansion = ctx.getExpander().expand(p);
			proofs.remove(p);
			proofs.addAll(expansion);
			return expansionStep(ctx);
		}
	}

	public static class ExpansionStepResult<T> {

		private final boolean finished;
		private final T value;
		private final Assertion assertion;

		private ExpansionStepResult(boolean finished, T value, Assertion assertion) {
			this.finished = finished;
			this.value = value;
			this.assertion = assertion;
		}

		public static <T> ExpansionStepResult<T> failedResult() {
			return new ExpansionStepResult<T>(true, null, null);
		}

		public static <T> ExpansionStepResult<T> successResult(T value) {
			if (value == null) {
				throw new NullPointerException();
			}
			return new ExpansionStepResult(true, value, null);
		}

		public static <T> ExpansionStepResult<T> inProgressResult(Assertion assertion) {
			return new ExpansionStepResult<T>(false, null, assertion);
		}

		public final boolean isFinished() {
			return finished;
		}

		public final T getValue() {
			return value;
		}

		public final Assertion getAssertion() {
			return assertion;
		}

	}

	/*
	 * Interpretation step:
	 * P = mostLikely()
	 * if (P stable) {
	 *   I = interpretation(P);
	 *   if (I not null) {
	 *     return I
	 *     FINISHED
	 *   }
	 * }
	 * elseif (P has assertions) {
	 *   A = assertionsIn(P);
	 *   act on A;
	 * }
	 * else { // P unsolved, no assertions
	 *   PX = expand(P)
	 *   remove(P)
	 *   insertAll(PX)
	 * }
	 * 
	 * Act on Assertions As:
	 * for each A {
	 *   U = updateFromSolution(A);
	 * }
	 * 
	 * Can an assertion FAIL (e.g. timeout)
	 *   - in that case the proof should be removed
	 */

	protected final void prune() {
		pruner.prune(proofs);
	}

}
