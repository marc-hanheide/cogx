package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.time.Interval;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import java.util.List;

public interface ProofConvertor {

	public IntentionRecognitionResult proofToIntentionRecognitionResult(LogicalForm lf, ProofWithCost pwc, float probBound, Interval ival, List<ResolutionRequest> rrqs);

}
