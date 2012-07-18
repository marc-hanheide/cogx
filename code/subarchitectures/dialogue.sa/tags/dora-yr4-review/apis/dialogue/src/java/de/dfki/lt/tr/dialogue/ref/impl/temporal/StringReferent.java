package de.dfki.lt.tr.dialogue.ref.impl.temporal;

import de.dfki.lt.tr.dialogue.ref.Referent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import java.text.ParseException;

public class StringReferent
implements Referent {

	String content = null;
	dFormula f = null;

	public StringReferent(String s) throws ParseException {
		assert (s != null);

		content = s;

		f = parseContentString(s);
		if (f == null) {
			throw new ParseException("Failed to convert the string to a referent: \"" + s + "\"", 0);
		}
	}

	@Override
	public String toString() {
		return content;
	}

	@Override
	public dFormula toFormula() {
		return f;
	}

	@Override
	public boolean equals(Object o) {
		if (o instanceof StringReferent) {
			return toString().equals(((StringReferent) o).toString());
		}
		else {
			return false;
		}
	}

	private static dFormula parseContentString(String content) {
		if (content.startsWith("Click")) {
			// a click
			String[] coords = content.substring(6).split(" ");
			String s = coords[0] + "," + coords[1] + "," + coords[2];
			return new ModalFormula(-1, "coordinates", new ElementaryFormula(-1, s));
		}
		else {
			// a landmark
			return new ModalFormula(-1, "landmark", new ElementaryFormula(-1, content));
		}
	}

	@Override
	public int hashCode() {
		return content.hashCode();
	}

}
