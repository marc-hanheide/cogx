/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.bnj.ver3.dynamic;

import edu.ksu.cis.bnj.ver3.core.BeliefNetwork;
import edu.ksu.cis.bnj.ver3.core.BeliefNode;

/*
 * \file DynamicTyping.java
 * \author Jeffrey M. Barber
 */
public class DynamicTyping {

	public static boolean IsDynamic(BeliefNetwork bn, boolean OldDBN) {
		if (bn == null)
			return false;
		// Check the names for naming conventions [0],[t-1],[t]
		BeliefNode[] nodes = bn.getNodes();
		if (nodes.length == 0)
			return false;

		for (int i = 0; i < nodes.length; i++) {
			String name = nodes[i].getName();

			// name MUST be unique
			for (int k = i + 1; k < nodes.length; k++) {
				if (name.equals(nodes[k].getName()))
					return false;
			}

			boolean good = false;
			if (name.indexOf("[t]") >= 0) {

				good = true;

			}
			if (name.indexOf("[t-1]") >= 0) {
				if (good)
					good = false;
				else {

					// must have a Unique! zero
					int Count0 = 0;
					String search = UnRoll.simpleReplaceAll(name, "[t-1]",
					"[0]");

					if(OldDBN)
				    {
						for (int k = 0; k < nodes.length; k++) {
							if (search.equals(nodes[k].getName()))
								Count0++;
						}
				    }
				    else
				    {
				        Count0 = 1;
				    }
				    

					// MUST have a Unique OutGoing interface
					int CountT = 0;
					String search2 = UnRoll.simpleReplaceAll(name, "[t-1]",
							"[t]");
					for (int k = 0; k < nodes.length; k++) {
						if (search2.equals(nodes[k].getName()))
							CountT++;
					}
					
					// check uniqueness
					if (Count0 == 1 && CountT == 1)
						good = true;

					// must not have a parent of an [t]
					BeliefNode[] parents = bn.getParents(nodes[i]);
					for (int k = 0; k < parents.length && good; k++) {
						if (parents[k].getName().indexOf("[t]") >= 0) {
							good = false;
						}
					}
				}
			}
			if (name.indexOf("[0]") >= 0) {
				if (good)
					good = false;
				else {
					good = true;

					// can not depend on anything but zeros
					BeliefNode[] parents = bn.getParents(nodes[i]);
					for (int k = 0; k < parents.length && good; k++) {
						if (parents[k].getName().indexOf("[t]") >= 0) {
							good = false;
						}
						if (parents[k].getName().indexOf("[t-1]") >= 0) {
							good = false;
						}
					}
				}
			}
			if (!good)
				return false;
		}
		return true;
	}
}