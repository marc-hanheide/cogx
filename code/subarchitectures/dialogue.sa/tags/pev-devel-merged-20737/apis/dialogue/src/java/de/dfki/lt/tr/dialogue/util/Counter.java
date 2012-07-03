package de.dfki.lt.tr.dialogue.util;

import java.util.*;

/** A simple identifier generator (a "counter") for identifiers
 *  of the form NAME:SUB:NUM, where NAME is the counter name,
 *  SUB is a subcounter and NUM is a number.
 *
 * @author Miroslav Janicek
 */
public class Counter {

	protected String name;
	protected HashMap<String, Integer> subcounters = null;

	private final static int FIRST_INDEX = 1;
        private final static String DELIMITER = ":";

        /** Instantiate a new counter.
         *
         * @param _name counter name
         */
	public Counter(String _name) {
		name = _name;
		subcounters = new HashMap<String, Integer>();
	}

        /** Return a unique identifier for a given subcounter.
         *
         * @param subcounter
         * @return
         */
	public String inc(String subcounter) {
		
		Integer current = subcounters.get(subcounter);
		
		if (current == null) {
			current = new Integer(FIRST_INDEX);
		}
		else {
			current += 1;
		}
		subcounters.put(subcounter, current);

		return this.name + DELIMITER + subcounter + DELIMITER + current.toString();
	}
}
