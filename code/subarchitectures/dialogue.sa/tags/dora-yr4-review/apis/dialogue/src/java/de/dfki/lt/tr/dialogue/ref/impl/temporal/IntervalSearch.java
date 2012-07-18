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

package de.dfki.lt.tr.dialogue.ref.impl.temporal;

import de.dfki.lt.tr.dialogue.ref.impl.temporal.scorers.IntervalScorer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import org.apache.log4j.Logger;

public class IntervalSearch<T> {

	private Logger logger = Logger.getLogger("interval-search");

	public static final int ERROR_ID = -1;
	private ArrayList<TemporalInterval<T>> ivals = new ArrayList<TemporalInterval<T>>();

	public int newInterval(long begin, T content) {
		int id = ivals.size();
		ivals.add(new TemporalInterval<T>(begin, content));
		return id;
	}

	public void closeInterval(int id, long end) {
		ivals.get(id).setEnd(end);
	}

	public Map<T, Double> findMatches(long begin, long end, IntervalScorer scorer) {
		assert (begin < end);
		Map<T, Double> result = new HashMap<T, Double>();
		logger.debug("searching " + ivals.size() + " intervals, matching against (" + begin+ ", " + end + ") length=" + (end - begin));
		for (TemporalInterval<T> ival : ivals) {
			long overlap = ival.getOverlap(begin, end);
			logger.debug("  \"" + ival.getContent().toString() + "\" (" + ival.getBegin() + ", " + ival.getEnd() + ") overlap=" + overlap);
			if (overlap > 0) {
				double score = scorer.scoreInterval(ival);
				result.put(ival.getContent(), new Double(score));
				logger.debug("    --> okay, selected with score " + score);
			}
		}
		logger.debug("search finished");
		return result;
	}

};
