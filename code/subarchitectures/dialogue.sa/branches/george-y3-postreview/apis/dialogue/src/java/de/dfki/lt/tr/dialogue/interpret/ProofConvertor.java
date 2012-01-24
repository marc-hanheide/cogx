package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.infer.abducer.proof.ProofWithCost;
import java.util.List;

public interface ProofConvertor {

	public ReferenceResolutionRequestExtractor getReferenceResolutionRequestExtractor();

	public IntentionRecognitionResult proofToIntentionRecognitionResult(LogicalForm lf, ProofWithCost pwc, float probBound, TimeInterval ival, List<ReferenceResolutionRequest> rrqs);

}
