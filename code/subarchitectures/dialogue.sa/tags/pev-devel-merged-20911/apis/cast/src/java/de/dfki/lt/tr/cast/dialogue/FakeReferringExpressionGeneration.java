package de.dfki.lt.tr.cast.dialogue;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.cast.dialogue.FakeReferringExpressionGeneration.FakeGenerator;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationRequest;
import de.dfki.lt.tr.dialogue.production.ReferenceGenerationResult;
import de.dfki.lt.tr.dialogue.production.ReferringExpressionGenerator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

public class FakeReferringExpressionGeneration
extends AbstractReferringExpressionGenerationComponent<FakeGenerator> {

	@Override
	protected FakeGenerator initGenerator() {
		return new FakeGenerator();
	}

	public static class FakeGenerator implements ReferringExpressionGenerator {

		private final Map<String, String> props;
		private final String type;

		public FakeGenerator() {
			props = new HashMap<String, String>();
			props.put("color", "red");
			props.put("shape", "elongated");
			
			type = "object";
		}

		@Override
		public ReferenceGenerationResult generate(ReferenceGenerationRequest request, WorkingMemoryAddress requestAddr) {
			List<String> phrases = new LinkedList<String>();
			if (request.shortNP) {
				phrases.add(getShortNP(request.disabledProperties));
			}
			if (request.spatialRelation) {
				phrases.add("on the table in the kitchen");
			}

			return new ReferenceGenerationResult(requestAddr, join(" ", phrases));
		}

		private String getShortNP(List<String> disabledProps) {
			List<String> words = new LinkedList<String>();
/*			words.add("the");
			for (String key : props.keySet()) {
				if (disabledProps != null && !disabledProps.contains(key)) {
					words.add(props.get(key));
				}
			}
			words.add(type); */
			words.add("this");
			return join(" ", words);
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
