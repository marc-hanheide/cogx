package de.dfki.lt.tr.cast.dialogue;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.cast.dialogue.FakeReferringExpressionGeneration.FakeGenerator;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;

public class FakeReferringExpressionGeneration
extends AbstractReferringExpressionGenerationComponent<FakeGenerator> {

	@Override
	protected FakeGenerator initGenerator() {
		return new FakeGenerator();
	}

	public static class FakeGenerator implements ReferringExpressionGenerator {

		@Override
		public ReferenceGenerationResult generate(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr) {
			String category = "cornflakes";
			String relation = "on";
			String location = "the table in the kitchen";
			return new ReferenceGenerationResult(requestAddr, category, relation, location);
		}
		
	}
}
