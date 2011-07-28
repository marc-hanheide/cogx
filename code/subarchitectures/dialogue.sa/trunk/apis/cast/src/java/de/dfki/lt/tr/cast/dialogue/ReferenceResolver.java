package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ResolutionResult;

public interface ReferenceResolver {

	public ResolutionResult resolve(ResolutionRequest rr);

}
