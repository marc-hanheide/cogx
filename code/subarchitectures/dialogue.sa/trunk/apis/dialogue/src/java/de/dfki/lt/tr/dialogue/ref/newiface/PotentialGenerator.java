package de.dfki.lt.tr.dialogue.ref.newiface;

import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;

public interface PotentialGenerator {

	Potential getHypos(ResolutionRequest rr);

}
