package de.dfki.lt.tr.dialogue.parseselection;

import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;

public interface ParseSelector {

	public LogicalForm selectParse(PackedLFs plfs);

}
