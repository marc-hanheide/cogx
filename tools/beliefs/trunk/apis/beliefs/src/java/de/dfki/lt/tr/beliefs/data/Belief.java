/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.genericproxies.Content;
import de.dfki.lt.tr.beliefs.factories.ContentFactory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class Belief<T extends dBelief> extends GenericBelief<T, Content<?>> {

	public static <T2 extends dBelief> Belief<T2> create(
			Class<? extends T2> type, Ice.Object o) {
		return new Belief<T2>(type, o);
	}

	public static <T2 extends dBelief> Belief<T2> create(
			Class<? extends T2> type) {
		T2 internal = createDefault(type);
		return new Belief<T2>(type, internal);
	}

	protected Belief(Class<? extends T> class1, Ice.Object content) {
		super(class1, new ContentFactory(), content);
	}

}
