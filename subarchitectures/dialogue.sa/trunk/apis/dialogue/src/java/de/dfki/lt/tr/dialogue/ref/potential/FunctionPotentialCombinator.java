package de.dfki.lt.tr.dialogue.ref.potential;

import de.dfki.lt.tr.dialogue.ref.Referent;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;

public class FunctionPotentialCombinator
implements PotentialCombinator {

	MapPotential pot = null;
	ScoreFunction f = null;

	public FunctionPotentialCombinator(ScoreFunction f) {
		pot = new MapPotential();
		this.f = f;
	}

	public FunctionPotentialCombinator(ScoreFunction f, Potential p) {
		this(f);
		pot = new MapPotential(p);
	}

	public void addPotential(Potential p) {
		Map<Referent, Double> used = new HashMap<Referent, Double>(pot.asMap());

		// for all in pot
		Iterator<Entry<Referent, Double>> iter = pot.positiveElementsIterator();
		while (iter.hasNext()) {
			Entry<Referent, Double> e = iter.next();
			double this_score = e.getValue();
			double arg_score = p.getScore(e.getKey());
			double this_max = pot.getMaxScore();
			double arg_max = p.getMaxScore();
			pot.setScore(e.getKey(), f.apply(this_score, arg_score, this_max, arg_max));
		}
/*
		// for all in p
		Iterator<Entry<Referent, Double>> iter = p.positiveElementsIterator();
		while (iter.hasNext()) {
			Entry<Referent, Double> rs = iter.next();
			double left_score = pot.getScore(rs.getKey());
			double right_score = rs.getValue();
			double left_max = pot.getMaxScore();
			double right_max = p.getMaxScore();
			pot.setScore(rs.getKey(), f.apply(left_score, right_score, left_max, right_max));
			used.remove(rs.getKey());
		}

		// for all in (pot - p) ... i.e. those unused
		for (Referent ur : used.keySet()) {
			double left_score = pot.getScore(ur);
			double right_score = 0.0;
			double left_max = pot.getMaxScore();
			double right_max = p.getMaxScore();
			pot.setScore(ur, f.apply(left_score, right_score, left_max, right_max));
		}
 */
	}

	public Potential toPotential() {
		return pot;
	}

}
