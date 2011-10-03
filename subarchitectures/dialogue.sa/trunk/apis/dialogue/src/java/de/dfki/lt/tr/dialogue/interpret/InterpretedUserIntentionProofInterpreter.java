package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.proof.ProofInterpreter;
import java.util.List;

public class InterpretedUserIntentionProofInterpreter
implements ProofInterpreter<InterpretedUserIntention> {

	public InterpretedUserIntentionProofInterpreter() {
	}

	@Override
	public InterpretedUserIntention interpret(List<ModalisedAtom> matoms) {
		InterpretedUserIntention iui = new InterpretedUserIntention();
		
		return iui.isWellFormed() ? iui : null;
	}
	
}
