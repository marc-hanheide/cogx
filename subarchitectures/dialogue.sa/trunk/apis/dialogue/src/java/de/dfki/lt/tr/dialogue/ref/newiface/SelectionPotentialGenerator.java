// =================================================================
// Copyright (C) 2011 DFKI GmbH Talking Robots
// Miroslav Janicek (miroslav.janicek@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

package de.dfki.lt.tr.dialogue.ref.newiface;

import de.dfki.lt.tr.dialogue.ref.Constraint;
import de.dfki.lt.tr.dialogue.ref.EpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.ref.ResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.TimeConversions;
import de.dfki.lt.tr.dialogue.ref.newiface.scorers.IntervalScorer;
import de.dfki.lt.tr.dialogue.ref.newiface.scorers.MixtureScorer;
import java.util.List;
import java.util.ArrayList;
import org.apache.log4j.Logger;

public class SelectionPotentialGenerator
implements PotentialGenerator {

	private Logger logger = null;
	private IntervalSearch<Referent> selections = null;

	public SelectionPotentialGenerator(Logger logger, IntervalSearch<Referent> selections) {
		this.logger = logger;
		this.selections = selections;
	}

	public Potential getHypos(ResolutionRequest rr) {
		Potential pot = new MapPotential();

		List<EpistemicReferenceHypothesis> result = new ArrayList<EpistemicReferenceHypothesis>();

		long test_begin = TimeConversions.timePointToMillisLong(rr.ival.begin);
		long test_end = TimeConversions.timePointToMillisLong(rr.ival.end);

		if (test_end - test_begin < 1) {
			test_end = test_begin + 1;
		}
		assert (test_end - test_begin >= 1);

		logger.debug("interval endpoint was " + (System.currentTimeMillis() - test_end) + " ms ago");
		logger.debug("interval length is " + (test_end - test_begin) + " ms");

		IntervalScorer scorer = new MixtureScorer(test_begin, test_end, 0.3, 0.7);

		pot = new MapPotential(selections.findMatches(test_begin, test_end, scorer));
		pot = PotentialNormalisation.massNormalisedPotential(pot);

		logger.debug("got " + pot.asMap().size() + " items in the potential");

		return pot;
	}

};
