package de.dfki.lt.tr.dialogue.ref.potential;

import de.dfki.lt.tr.dialogue.ref.Referent;
import java.util.Iterator;
import java.util.Map.Entry;

public abstract class PotentialNormalisation {

	public static Potential massNormalisedPotential(Potential p) {
		MapPotential result = new MapPotential();
		Iterator<Entry<Referent, Double>> iter = p.positiveElementsIterator();
		double total_mass = 0.0;
		while (iter.hasNext()) {
			Entry<Referent, Double> e = iter.next();
			result.setScore(e.getKey(), e.getValue());
			total_mass += e.getValue();
		}

		// now adjust the scores normalise
		iter = result.positiveElementsIterator();
		assert (total_mass > 0.0 || !iter.hasNext());

		while (iter.hasNext()) {
			Entry<Referent, Double> e = iter.next();
			result.setScore(e.getKey(), e.getValue() / total_mass);
		}

		return result;
	}

	public static Potential absNormalisedPotential(Potential p, double max) {
		MapPotential result = new MapPotential();
		if (max > 0.0) {
			Iterator<Entry<Referent, Double>> iter = p.positiveElementsIterator();
			while (iter.hasNext()) {
				Entry<Referent, Double> e = iter.next();
				result.setScore(e.getKey(), e.getValue() / max);
			}
			return result;
		}
		else {
			return new MapPotential(p);
		}
	}

	public static Potential maxNormalisedPotential(Potential p) {
		return absNormalisedPotential(p, p.getMaxScore());
	}

}
