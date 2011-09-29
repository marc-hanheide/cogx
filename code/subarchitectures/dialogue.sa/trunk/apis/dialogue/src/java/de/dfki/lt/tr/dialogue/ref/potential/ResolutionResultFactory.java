package de.dfki.lt.tr.dialogue.ref.potential;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.dialogue.ref.Referent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionResult;
import de.dfki.lt.tr.dialogue.util.EpistemicStatusFactory;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map.Entry;

public class ResolutionResultFactory {

	public static ReferenceResolutionResult potentialToResolutionResult(String nom, Potential pot, WorkingMemoryAddress origin, String method) {
		return new ReferenceResolutionResult(nom, origin, method, potentialToEpistemicRefHypos(pot));
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

			EpistemicReferenceHypothesis hypo = new EpistemicReferenceHypothesis(EpistemicStatusFactory.newPrivateEpistemicStatus("self"), f, score);
			result.add(hypo);
		}

		return result;
	}

}
