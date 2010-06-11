package eu.cogx.beliefproxies.factories.logicalcontent;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.logicalcontent.FormulaProxy;

public class FormulaFactory implements
		ProxyFactory<Proxy<? extends dFormula>> {

	@Override
	public FormulaProxy<dFormula> create(Ice.Object pd) {
		return new FormulaProxy<dFormula>(dFormula.class, (dFormula) pd);
	}

	@Override
	public FormulaProxy<dFormula> create(Proxy<? extends Ice.Object> proxy) {
		return create(proxy.get());
	}

}
