/**
 * 
 */
package eu.cogx.beliefproxies.proxies.distributions;

import Ice.Object;
import eu.cogx.beliefproxies.factories.logicalcontent.FormulaValuesFactory;
import eu.cogx.beliefproxies.proxies.logicalcontent.FormulaValuesProxy;

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
