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

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.util.LFPacking;
import de.dfki.lt.tr.dialogue.util.LFUtils;

public abstract class RandomParseSelection {

	static boolean logging = true;

	static LFPacking packingTool = new LFPacking();

	/**
	 * Extract a (random) logical form from the packed representation.
	 *
	 * @param plf   The packed logical form
	 * @return logical form, or null if none could be extracted
	 */
	public static LogicalForm extractLogicalForm(PackedLFs plf) {
		LogicalForm[] lfs = packingTool.unpackPackedLogicalForm(plf);
		log("we've got " + lfs.length + " alternatives");
		for (int i = 0; i < lfs.length; i++) {
			LogicalForm lf = lfs[i];
			if (!LFUtils.lfNominalGetFeature(lf.root, "Mood").isEmpty()) {
				return lf;
			}
		}
		return null;
	}

	private static void log(String str) {
		if (logging)
			System.out.println("\033[32m[ParseSelection]\t" + str + "\033[0m");
	}

}
