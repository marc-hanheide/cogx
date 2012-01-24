package de.dfki.lt.tr.dialogue.production;

import cast.cdl.WorkingMemoryAddress;

public interface ReferringExpressionGenerator {

	public ReferenceGenerationResult generate(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr);

}
