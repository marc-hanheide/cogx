// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
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

package de.dfki.lt.tr.dialogue.interpret;

import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

final class EpistemicStatusComparator implements Comparator<EpistemicStatus> {

	@Override
	public int compare(EpistemicStatus e1, EpistemicStatus e2) {

		if (e2 == null) {
			return -1;
		}
		if (e1 == null) {
			return +1;
		}

		if (e1 instanceof PrivateEpistemicStatus) {
			if (e2 instanceof PrivateEpistemicStatus) {
				PrivateEpistemicStatus p1 = (PrivateEpistemicStatus)e1;
				PrivateEpistemicStatus p2 = (PrivateEpistemicStatus)e2;
				return p1.agent.compareTo(p2.agent);
			}
			else {
				return +1;
			}
		}

		else if (e1 instanceof AttributedEpistemicStatus) {
			if (e2 instanceof PrivateEpistemicStatus) {
				return -1;
			}
			else if (e2 instanceof AttributedEpistemicStatus) {
				AttributedEpistemicStatus a1 = (AttributedEpistemicStatus)e1;
				AttributedEpistemicStatus a2 = (AttributedEpistemicStatus)e2;
				int tmp = a1.agent.compareTo(a2.agent);

				if (tmp == 0) {
					List<String> ags1 = new ArrayList<String>(a1.attribagents);
					List<String> ags2 = new ArrayList<String>(a2.attribagents);
					Collections.sort(ags1);
					Collections.sort(ags2);
					if (ags1.containsAll(ags2)) {
						if (ags2.containsAll(ags1)) {
							return 0;
						}
						return -1;
					}
					return +1;
				}
				else {
					return tmp;
				}
			}
			else {
				return +1;
			}
		}

		else if (e1 instanceof SharedEpistemicStatus) {
			if (e2 instanceof SharedEpistemicStatus) {
				SharedEpistemicStatus s1 = (SharedEpistemicStatus)e1;
				SharedEpistemicStatus s2 = (SharedEpistemicStatus)e2;
				List<String> ags1 = new ArrayList<String>(s1.cgagents);
				List<String> ags2 = new ArrayList<String>(s2.cgagents);
				Collections.sort(ags1);
				Collections.sort(ags2);
				if (ags1.containsAll(ags2)) {
					if (ags2.containsAll(ags1)) {
						return 0;
					}
					return -1;
				}
				return +1;
			}
			else {
				return -1;
			}
		}
		return -1;  // shouldn't really happen
	}
}