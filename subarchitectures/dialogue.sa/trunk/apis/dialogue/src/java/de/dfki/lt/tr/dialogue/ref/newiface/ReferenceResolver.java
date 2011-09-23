package de.dfki.lt.tr.dialogue.ref.newiface;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;

public interface ReferenceResolver {

	public ReferenceResolutionResult resolve(ReferenceResolutionRequest rr);

}
