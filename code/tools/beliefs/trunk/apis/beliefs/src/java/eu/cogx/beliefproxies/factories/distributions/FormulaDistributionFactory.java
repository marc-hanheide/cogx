/**
 * 
 */
package eu.cogx.beliefproxies.factories.distributions;

import java.util.LinkedList;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.distributions.BasicProbDistributionProxy;
import eu.cogx.beliefproxies.proxies.distributions.FormulaDistributionProxy;

/**
 * @author marc
 *
 */
public class FormulaDistributionFactory implements ProxyFactory<BasicProbDistributionProxy<?>> {

	public static FormulaDistributionProxy create(String key) {
		FormulaDistributionFactory factory = new FormulaDistributionFactory();
		BasicProbDistribution bpd = new BasicProbDistribution(key, new FormulaValues(new LinkedList<FormulaProbPair>()));
		return factory.create(bpd);
	}
	
	@Override
	public FormulaDistributionProxy create(Object pd) {
		return new FormulaDistributionProxy(pd);
	}

	@Override
	public FormulaDistributionProxy create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
