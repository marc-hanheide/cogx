/**
 * @author Marc Hanheide (marc@hanheide.de)
 */
package de.dfki.lt.tr.beliefs.factories;

import java.util.HashMap;

import Ice.Object;

import de.dfki.lt.tr.beliefs.data.Content;
import de.dfki.lt.tr.beliefs.data.IndependentDistribution;
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

/**
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 */
public class IndependentDistributionFactory  implements DefaultProxyFactory<IndependentDistribution<? extends Content<?>>> {

	public IndependentDistribution<? extends Content<?>> create() {
		return create(new CondIndependentDistribs(new HashMap<String, ProbDistribution>()));
	}

	@Override
	public IndependentDistribution<? extends Content<?>> create(Ice.Object pd) {
		return IndependentDistribution.create(new ContentFactory(), pd);
	}

	@Override
	public IndependentDistribution<? extends Content<?>> create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
