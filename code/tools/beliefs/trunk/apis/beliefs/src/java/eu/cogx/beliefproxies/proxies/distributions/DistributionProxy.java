package eu.cogx.beliefproxies.proxies.distributions;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import eu.cogx.beliefproxies.proxies.Proxy;

public abstract class DistributionProxy<T extends ProbDistribution> extends
		Proxy<T> {

	public DistributionProxy(Class<? extends T> class1, Object content) {
		super(class1, content);
	}

}
