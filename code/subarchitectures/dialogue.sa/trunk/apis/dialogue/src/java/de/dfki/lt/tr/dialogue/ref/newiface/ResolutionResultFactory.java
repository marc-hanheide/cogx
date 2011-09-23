package de.dfki.lt.tr.dialogue.ref.newiface;

import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;

public class ResolutionResultFactory {

	public static ReferenceResolutionResult potentialToResolutionResult(String nom, Potential pot) {
		return new ReferenceResolutionResult(nom, potentialToEpistemicRefHypos(pot));
	}

	private static List<EpistemicReferenceHypothesis> potentialToEpistemicRefHypos(Potential pot) {
		List<EpistemicReferenceHypothesis> result = new ArrayList<EpistemicReferenceHypothesis>();

		Iterator<Entry<Referent, Double>> iter = pot.positiveElementsIterator();
		while (iter.hasNext()) {
			Entry<Referent, Double> elem = iter.next();

			dFormula f = elem.getKey().toFormula();
			double score = elem.getValue();

			assert (f != null);
			assert (score > 0.0);

			EpistemicReferenceHypothesis hypo = new EpistemicReferenceHypothesis(new PrivateEpistemicStatus("self"), f, score);
			result.add(hypo);
		}

		return result;
	}

}
