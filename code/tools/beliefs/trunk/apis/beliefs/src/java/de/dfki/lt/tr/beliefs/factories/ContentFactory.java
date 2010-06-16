/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import Ice.Object;
import de.dfki.lt.tr.beliefs.data.Content;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 *
 */
public class ContentFactory extends AbstractProxyFactory<Content<?>> {

	@Override
	public Content<?> create(Object pd) {
		return Content.create(ProbDistribution.class, pd);
	}

}
