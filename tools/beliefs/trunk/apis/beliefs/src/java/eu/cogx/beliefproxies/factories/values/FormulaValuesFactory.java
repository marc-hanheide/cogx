/**
 * 
 */
package eu.cogx.beliefproxies.factories.values;

import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.values.FormulaValuesProxy;

/**
 * @author marc
 * 
 */
public class FormulaValuesFactory implements
		ProxyFactory<FormulaValuesProxy<?>> {

	@Override
	public FormulaValuesProxy<?> create(Ice.Object pd) {
		return new FormulaValuesProxy<FormulaFactory>(new FormulaFactory(), pd);
	}

	@Override
	public FormulaValuesProxy<?> create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
