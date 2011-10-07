package de.dfki.lt.tr.cast.dialogue;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.cast.dialogue.FakeReferringExpressionGeneration.FakeGenerator;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class FakeReferringExpressionGeneration
extends AbstractReferringExpressionGenerationComponent<FakeGenerator> {

	@Override
	protected FakeGenerator initGenerator() {
		return new FakeGenerator();
	}

	public static class FakeGenerator implements ReferringExpressionGenerator {

		@Override
		public ReferenceGenerationResult generate(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr) {
			List<String> phrases = new LinkedList<String>();
			if (request.shortNP) {
				phrases.add("the cornflakes");
			}
			if (request.spatialRelation) {
				phrases.add("on the table in the kitchen");
			}

			return new ReferenceGenerationResult(requestAddr, join(" ", phrases));
		}

		public static String join(String separator, List<String> args) {
			StringBuilder sb = new StringBuilder();
			if (args != null) {
				Iterator<String> iter = args.iterator();
				while (iter.hasNext()) {
					sb.append(iter.next());
					if (iter.hasNext()) {
						sb.append(separator);
					}
				}
			}
			return sb.toString();
		}

	}
}
