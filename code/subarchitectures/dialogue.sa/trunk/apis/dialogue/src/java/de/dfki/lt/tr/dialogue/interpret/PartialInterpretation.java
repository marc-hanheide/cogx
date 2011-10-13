package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.proof.Proof;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import de.dfki.lt.tr.infer.abducer.proof.ProofSet;
import java.util.LinkedList;
import java.util.List;
import org.apache.log4j.Logger;

public class PartialInterpretation<T> {

	private final ProofSet proofs;
	private final TerminationCondition<T> cond;
	private final List<T> interpretations;
	private final double priorConfidence;

	private PartialInterpretation(Logger logger, ProofPruner pruner, Proof initialProof, double priorConfidence, TerminationCondition<T> cond) {
		proofs = new ProofSet(logger, pruner, initialProof);
		this.cond = cond;
		interpretations = new LinkedList<T>();
		this.priorConfidence = priorConfidence;
	}

	public static PartialInterpretation fromModalisedAtom(Logger logger, ModalisedAtom matom, ProofPruner pruner, double priorConfidence, TerminationCondition cond) {
		return new PartialInterpretation(logger, pruner, new Proof(matom), priorConfidence, cond);
	}

	public ProofSet getProofSet() {
		return proofs;
	}

	public List<T> getInterpretations() {
		return interpretations;
	}

	public boolean addInterpretation(T ipr) {
		interpretations.add(ipr);
		return cond.reached(this);
	}

	public double getPriorConfidence() {
		return priorConfidence;
	}

}
