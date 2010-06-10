/**
 * 
 */
package eu.cogx.beliefproxies.factories.beliefs;

import java.util.HashMap;

import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import Ice.Object;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.history.AbstractBeliefHistory;
import eu.cogx.beliefproxies.proxies.beliefs.CondIndepedentFormulaBeliefProxy;
import eu.cogx.beliefproxies.proxies.distributions.BasicProbDistributionProxy;
import eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy;

/**
 * @author marc
 * 
 */
public class CondIndependentFormulaBeliefFactory<T extends dBelief> implements
		ProxyFactory<CondIndepedentFormulaBeliefProxy<? extends T>> {

	private final Class<? extends T> beliefType;

	/**
	 * @param beliefType
	 */
	public CondIndependentFormulaBeliefFactory(Class<? extends T> beliefType) {
		super();
		this.beliefType = beliefType;
	}
	
	public static <T2 extends dBelief> CondIndepedentFormulaBeliefProxy<T2> create(Class<? extends T2> type, String beliefType, String id) throws InstantiationException, IllegalAccessException {
		CondIndependentFormulaBeliefFactory<T2> fact = new CondIndependentFormulaBeliefFactory<T2>(type);
		T2 belief = type.newInstance();
		belief.id=id;
		belief.content = new CondIndependentDistribs(new HashMap<String, ProbDistribution>());
		belief.estatus = new PrivateEpistemicStatus("robot");
		belief.frame = new AbstractFrame();
		belief.hist = new AbstractBeliefHistory();
		belief.type = beliefType;
		return fact.create(belief);
	}

	@Override
	public CondIndepedentFormulaBeliefProxy<T> create(Object pd) {
		return new CondIndepedentFormulaBeliefProxy<T>(beliefType, pd);
	}

	@Override
	public CondIndepedentFormulaBeliefProxy<T> create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
