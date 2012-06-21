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

package de.dfki.lt.tr.dialogue.ref.impl.temporal.scorers;

import de.dfki.lt.tr.dialogue.ref.impl.temporal.TemporalInterval;

public class EndpointScorer<T> implements IntervalScorer<T> {

	private long begin;
	private long end;
	private double weight;

	public EndpointScorer(long _begin, long _end, double _weight) {
		begin = _begin;
		end = _end;

		weight = _weight;
		assert (0.0 <= weight && weight <= 1.0);
	}

	@Override
	public double scoreInterval(TemporalInterval<T> ival) {

		long overlap = ival.getOverlap(begin, end);

		if (overlap > 0) {
			if (ival.getEnd() >= end || ival.getEnd() == TemporalInterval.NOW) {
				return weight;
			}
			else {
				return (1.0 - weight);
			}
		}
		else {
			return 0.0;
		}
	}

}
