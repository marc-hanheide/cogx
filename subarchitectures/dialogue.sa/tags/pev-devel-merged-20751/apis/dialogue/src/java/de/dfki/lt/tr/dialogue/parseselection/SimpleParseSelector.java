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

package de.dfki.lt.tr.dialogue.parseselection;

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.util.LFPacking;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import org.apache.log4j.Logger;

public class SimpleParseSelector implements ParseSelector {

	private Logger logger;
	private static LFPacking packingTool = new LFPacking();

	public SimpleParseSelector() {
		logger = null;
	}

	public SimpleParseSelector(Logger logger) {
		this.logger = logger;
	}

	public void setLogger(Logger logger) {
		this.logger = logger;
	}

	/**
	 * Extract a (random) logical form from the packed representation.
	 *
	 * @param plf   The packed logical form
	 * @return logical form, or null if none could be extracted
	 */
	@Override
	public LogicalForm selectParse(PackedLFs plf) {
		List<LogicalForm> lfs = new LinkedList<LogicalForm>(Arrays.asList(packingTool.unpackPackedLogicalForm(plf)));
		List<LogicalForm> filteredLfs = copyFilterLogicalForms(lfs);
		log("we've got " + lfs.size() + " alternatives, " + filteredLfs.size() + " after filtering");

		if (filteredLfs.size() > 0) {
			LogicalForm result = filteredLfs.get(0);
			result.preferenceScore = plf.phonStringConfidence;  // XXX hack
			return result;
		}
		else {
			return null;
		}
	}

	public List<LogicalForm> copyFilterLogicalForms(List<LogicalForm> lfs) {
		List<LogicalForm> newLfs = new LinkedList<LogicalForm>();
		for (LogicalForm lf : lfs) {
			boolean ok = true;
			if (lf.root.sort.equals("marker") || lf.root.sort.equals("greeting") || lf.root.sort.equals("closing")) {
				// ok
			}
			else if (lf.root.sort.startsWith("q-")) {
				// fine as well
			}
			else if (lf.root.sort.startsWith("e-")) {
				// fine as well
			}
			else {
				if (LFUtils.lfNominalGetFeature(lf.root, "Mood").isEmpty()) {
					ok = false;
				}
			}
			if (ok) {
				newLfs.add(lf);
			}

			log((ok ? "YES " : "NO  ") + LFUtils.lfToString(lf));
		}
		return newLfs;
	}

	private void log(String str) {
		if (logger != null) {
			logger.debug(str);
		}
	}

}
