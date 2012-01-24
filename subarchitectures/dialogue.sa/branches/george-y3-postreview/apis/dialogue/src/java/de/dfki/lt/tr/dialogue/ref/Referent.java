package de.dfki.lt.tr.dialogue.ref;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;

public interface Referent {

	/**
	 * Convert the referent to a dFormula so that it can be included
	 * in the intention.
	 *
	 * @return corresponding dFormula
	 */
	public dFormula toFormula();

}
