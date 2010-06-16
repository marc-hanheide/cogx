/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.data;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.data.genericproxies.Content;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericIndependentDistribution;
import de.dfki.lt.tr.beliefs.factories.ContentFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentDistribution extends
		GenericIndependentDistribution<Content<ProbDistribution>> {
	public static  IndependentDistribution create(Ice.Object pd) {
		return new IndependentDistribution(pd);
	}

	protected IndependentDistribution(Object content) {
		super(new ContentFactory(), content);
	}

}
