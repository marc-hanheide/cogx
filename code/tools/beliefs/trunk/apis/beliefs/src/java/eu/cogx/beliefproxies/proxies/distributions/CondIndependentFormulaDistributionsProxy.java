/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import eu.cogx.beliefproxies.factories.distributions.FormulaDistributionFactory;
import eu.cogx.beliefproxies.proxies.logicalcontent.FormulaValuesProxy;

/**
 * @author marc
 * 
 */
public class CondIndependentFormulaDistributionsProxy extends
		CondIndependentDistributionProxy<FormulaDistributionFactory> {

	public CondIndependentFormulaDistributionsProxy(Object content) {
		super(new FormulaDistributionFactory(), content);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy
	 * #get(java.lang.Object)
	 */
	@Override
	public FormulaDistributionProxy get(java.lang.Object arg0) {
		ProbDistribution entry=_content.distribs.get(arg0);
		if (entry==null)
			return null;
		return factory.create(entry);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy
	 * #get(java.lang.Object)
	 */
		public FormulaValuesProxy<?> getDistribution(java.lang.Object arg0) {
		FormulaDistributionProxy fdp = get(arg0);
		if (fdp==null)
			return null;
		return fdp.getDistribution();
	}

	public FormulaDistributionProxy initFeature(String string) {
		if (_content.distribs.containsKey(string))
			return this.get(string);
		else {
			FormulaDistributionProxy newOne = FormulaDistributionFactory.create(string);
			this.put(string, newOne);
			return newOne;
		}
		
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy
	 * #put(java.lang.String,
	 * eu.cogx.beliefproxies.proxies.distributions.DistributionProxy)
	 */
	@Override
	public FormulaDistributionProxy put(String arg0,
			DistributionProxy<? extends ProbDistribution> arg1) {
		ProbDistribution old = _content.distribs.put(arg0, arg1.get());
		if (old == null) 
			return null;
		else
			return factory.create(old);

	}



}
