/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import java.util.Map.Entry;

import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import Ice.Object;
import eu.cogx.beliefproxies.factories.values.FormulaValuesFactory;
import eu.cogx.beliefproxies.proxies.values.FormulaValuesProxy;

/**
 * @author marc
 * 
 */
public class FormulaDistributionProxy extends
		BasicProbDistributionProxy<FormulaValuesFactory> {

	/**
	 * @param class1
	 * @param content
	 */
	public FormulaDistributionProxy(Object content) {
		super(new FormulaValuesFactory(), content);

	}

	@Override
	public FormulaValuesProxy<?> getDistribution() {
		return factory.create(_content.values);
	}



}
