package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.proof.Proof;
import de.dfki.lt.tr.infer.abducer.proof.ProofPruner;
import de.dfki.lt.tr.infer.abducer.proof.ProofSet;
import org.apache.log4j.Logger;

public class PartialInterpretation {

	private final ProofSet proofs;
	private final TerminationCondition cond;

	private PartialInterpretation(Logger logger, ProofPruner pruner, Proof initialProof, TerminationCondition cond) {
		proofs = new ProofSet(logger, pruner, initialProof);
		this.cond = cond;
	}

	public static PartialInterpretation fromModalisedAtom(Logger logger, ModalisedAtom matom, ProofPruner pruner, TerminationCondition cond) {
		return new PartialInterpretation(logger, pruner, new Proof(matom), cond);
	}

	public ProofSet getProofSet() {
		return proofs;
	}

}
