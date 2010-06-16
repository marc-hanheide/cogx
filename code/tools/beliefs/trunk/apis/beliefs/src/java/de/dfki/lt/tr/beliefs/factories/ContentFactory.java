/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.data.genericproxies.Content;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class ContentFactory extends AbstractProxyFactory<Content<ProbDistribution>> {

	@Override
	public Content<ProbDistribution> create(Object pd) {
		return Content.create(ProbDistribution.class, pd);
	}

}
