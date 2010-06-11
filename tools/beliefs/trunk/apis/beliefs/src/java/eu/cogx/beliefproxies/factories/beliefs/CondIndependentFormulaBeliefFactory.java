/**
 * 
 */
package eu.cogx.beliefproxies.factories.beliefs;

import java.util.HashMap;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.framing.TemporalInterval;
import de.dfki.lt.tr.beliefs.slice.history.AbstractBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;
import eu.cogx.beliefproxies.proxies.beliefs.CondIndependentFormulaBeliefProxy;

/**
 * @author marc
 * 
 */
public class CondIndependentFormulaBeliefFactory<T extends dBelief> implements
		ProxyFactory<CondIndependentFormulaBeliefProxy<? extends T>> {

	public static <T2 extends dBelief> CondIndependentFormulaBeliefProxy<T2> create(Class<? extends T2> type, String beliefType, String id) throws InstantiationException, IllegalAccessException {
		CondIndependentFormulaBeliefFactory<T2> fact = new CondIndependentFormulaBeliefFactory<T2>(type);
		T2 belief = type.newInstance();
		belief.id=id;
		belief.content = new CondIndependentDistribs(new HashMap<String, ProbDistribution>());
		belief.estatus = new PrivateEpistemicStatus("robot");
		belief.frame = new SpatioTemporalFrame("here", new TemporalInterval());
		belief.hist = new AbstractBeliefHistory();
		belief.type = beliefType;
		return fact.create(belief);
	}

	private final Class<? extends T> beliefType;
	
	/**
	 * @param beliefType
	 */
	public CondIndependentFormulaBeliefFactory(Class<? extends T> beliefType) {
		super();
		this.beliefType = beliefType;
	}

	@Override
	public CondIndependentFormulaBeliefProxy<T> create(Object pd) {
		return new CondIndependentFormulaBeliefProxy<T>(beliefType, pd);
	}

	@Override
	public CondIndependentFormulaBeliefProxy<T> create(Proxy<? extends Object> proxy) {
		return create(proxy.get());
	}

}
