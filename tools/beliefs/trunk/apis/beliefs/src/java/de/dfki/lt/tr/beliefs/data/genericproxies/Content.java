package de.dfki.lt.tr.beliefs.data.genericproxies;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/** a specialisation of a Proxy for probability distributions
 * @author Marc Hanheide (marc@hanheide.de)
 *
 * @param <T> the encapsulated type
 */
public class Content<T extends ProbDistribution> extends
		Proxy<T> {

	public static <T2 extends ProbDistribution>  Content<T2> create(Class<? extends T2> type, Object pd) {
		return new Content<T2>(type, pd);
	}

	protected Content(Class<? extends T> class1, Object content) {
		super(class1, content);
	}

}
