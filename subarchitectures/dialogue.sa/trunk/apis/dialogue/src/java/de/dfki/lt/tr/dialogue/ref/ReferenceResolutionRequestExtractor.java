package de.dfki.lt.tr.dialogue.ref;

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;

public interface ReferenceResolutionRequestExtractor {

	public ReferenceResolutionRequest extractReferenceResolutionRequest(LogicalForm lf, String nomvar, TimeInterval ival);

}
