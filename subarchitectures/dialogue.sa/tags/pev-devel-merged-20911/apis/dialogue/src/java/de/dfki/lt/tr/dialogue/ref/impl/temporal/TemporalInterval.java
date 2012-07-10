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

public class TemporalInterval<T> {

	public static final long NOW = -1;

	private T content;
	private long begin;
	private long end = NOW;

	public TemporalInterval(long _begin, T _content) {
		assert (begin >= 0);
		begin = _begin;
		content = _content;
	}

	public void setEnd(long _end) {
		assert (begin < _end);
		end = _end;
	}

	public T getContent() {
		return content;
	}

	public long getBegin() {
		return begin;
	}

	public long getEnd() {
		return end;
	}

	/**
	 * Return the number of time units in which the interval
	 * overlaps with the interval specified as its argument.
	 * The argument must be a closed, non-empty interval.
	 */
	public long getOverlap(long mask_begin, long mask_end) {
		assert (mask_end != NOW);
		assert (mask_begin < mask_end);

		long endpoint = 0;
		if (end == NOW) {
			endpoint = mask_end;
		}
		else {
			endpoint = Math.min(mask_end, end);
		}

		long overlap = endpoint - Math.max(mask_begin, begin);
		return Math.max(0, overlap);
	}

};
