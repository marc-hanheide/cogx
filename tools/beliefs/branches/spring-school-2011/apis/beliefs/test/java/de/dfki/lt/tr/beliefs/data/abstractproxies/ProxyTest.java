package de.dfki.lt.tr.beliefs.data.abstractproxies;

import java.util.HashMap;

import org.junit.Test;

import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.factories.DistributionFactory;
import de.dfki.lt.tr.beliefs.factories.IndependentDistributionFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;


public class ProxyTest {
	/** A bare initialized belief can be given an identifier */
	@Test
	public void test() {
		Distribution<ProbDistribution> s = Proxy.create(DistributionFactory.get(), new CondIndependentDistribs(new HashMap<String, ProbDistribution>()));
		Proxy.create(IndependentDistributionFactory.get(), s);
		s.getAs(IndependentDistributionFactory.get()).clear();
	} // end test

}
