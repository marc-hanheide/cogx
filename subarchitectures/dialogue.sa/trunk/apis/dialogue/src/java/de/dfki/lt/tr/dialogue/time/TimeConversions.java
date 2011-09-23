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

package de.dfki.lt.tr.dialogue.time;

import de.dfki.lt.tr.dialogue.slice.time.TimePoint;

public class TimeConversions {

	/**
	 * Convert a TimePoint to milliseconds.
	 * @param tp the TimePoint
	 * @return milliseconds
	 */
	public static long timePointToMillisLong(TimePoint tp) {
		return tp.msec;
	}

	/**
	 * Convert seconds (as a double) to milliseconds (as a long).
	 * @param sec seconds
	 * @return milliseconds
	 */
	public static long secDoubleToMillisLong(double sec) {
		return (long) (sec * 1000);
	}

};
