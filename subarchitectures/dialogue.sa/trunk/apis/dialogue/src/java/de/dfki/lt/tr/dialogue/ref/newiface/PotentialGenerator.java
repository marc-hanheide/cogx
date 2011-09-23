package de.dfki.lt.tr.dialogue.ref.newiface;

import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;

public interface PotentialGenerator {

	Potential getHypos(ReferenceResolutionRequest rr);

}
