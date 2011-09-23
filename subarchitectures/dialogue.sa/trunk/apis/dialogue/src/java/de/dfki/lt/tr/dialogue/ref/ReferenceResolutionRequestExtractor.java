package de.dfki.lt.tr.dialogue.ref;

import de.dfki.lt.tr.dialogue.slice.lf.LFNominal;
import de.dfki.lt.tr.dialogue.time.TimeInterval;

public interface ReferenceResolutionRequestExtractor {

	public ReferenceResolutionRequest extractReferenceResolutionRequest(LFNominal lf, TimeInterval ival);

}
