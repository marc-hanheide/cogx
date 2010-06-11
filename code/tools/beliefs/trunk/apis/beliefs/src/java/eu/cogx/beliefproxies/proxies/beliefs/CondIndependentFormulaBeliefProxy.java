/**
 * 
 */
package eu.cogx.beliefproxies.proxies.beliefs;

import java.util.Collection;
import java.util.Map;
import java.util.Set;
import java.util.Map.Entry;

import Ice.Object;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.beliefproxies.factories.distributions.CondIndependentFormulaDistributionFactory;
import eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy;
import eu.cogx.beliefproxies.proxies.distributions.DistributionProxy;
import eu.cogx.beliefproxies.proxies.distributions.FormulaDistributionProxy;
import eu.cogx.beliefproxies.proxies.logicalcontent.FormulaValuesProxy;

/**
 * @author marc
 * 
 */
public class CondIndependentFormulaBeliefProxy<T extends dBelief> extends
		BeliefProxy<T, CondIndependentFormulaDistributionFactory> implements Map<String, DistributionProxy<? extends ProbDistribution>> {

	public CondIndependentFormulaBeliefProxy(Class<? extends T> class1,
			Object content) {
		super(class1, new CondIndependentFormulaDistributionFactory(), content);
	}

	/**
	 * 
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#clear()
	 */
	public void clear() {
		getContent().clear();
	}

	/**
	 * @param arg0
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#containsKey(java.lang.Object)
	 */
	public boolean containsKey(java.lang.Object arg0) {
		return getContent().containsKey(arg0);
	}

	/**
	 * @param arg0
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#containsValue(java.lang.Object)
	 */
	public boolean containsValue(java.lang.Object arg0) {
		return getContent().containsValue(arg0);
	}

	/**
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#entrySet()
	 */
	public Set<Entry<String, DistributionProxy<? extends ProbDistribution>>> entrySet() {
		return getContent().entrySet();
	}

	/**
	 * @param arg0
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy#get(java.lang.Object)
	 */
	public FormulaDistributionProxy get(java.lang.Object arg0) {
		return getContent().get(arg0);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see eu.cogx.beliefproxies.proxies.beliefs.BeliefProxy#getContent()
	 */
	@Override
	public CondIndependentFormulaDistributionsProxy getContent() {
		return contentFactory.create(_content.content);
	}

	/**
	 * @param arg0
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy#getDistribution(java.lang.Object)
	 */
	public FormulaValuesProxy<?> getDistribution(java.lang.Object arg0) {
		return getContent().getDistribution(arg0);
	}

	/**
	 * @param string
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy#initFeature(java.lang.String)
	 */
	public FormulaDistributionProxy initFeature(String string) {
		return getContent().initFeature(string);
	}

	/**
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#isEmpty()
	 */
	public boolean isEmpty() {
		return getContent().isEmpty();
	}

	/**
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#keySet()
	 */
	public Set<String> keySet() {
		return getContent().keySet();
	}

	/**
	 * @param arg0
	 * @param arg1
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentFormulaDistributionsProxy#put(java.lang.String, eu.cogx.beliefproxies.proxies.distributions.DistributionProxy)
	 */
	public FormulaDistributionProxy put(String arg0,
			DistributionProxy<? extends ProbDistribution> arg1) {
		return getContent().put(arg0, arg1);
	}

	/**
	 * @param arg0
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#putAll(java.util.Map)
	 */
	public void putAll(
			Map<? extends String, ? extends DistributionProxy<? extends ProbDistribution>> arg0) {
		getContent().putAll(arg0);
	}

	/**
	 * @param arg0
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#remove(java.lang.Object)
	 */
	public DistributionProxy<? extends ProbDistribution> remove(
			java.lang.Object arg0) {
		return getContent().remove(arg0);
	}

	/**
	 * @return
	 * @see eu.cogx.beliefproxies.proxies.distributions.CondIndependentDistributionProxy#size()
	 */
	public int size() {
		return getContent().size();
	}

	@Override
	public Collection<DistributionProxy<? extends ProbDistribution>> values() {
		return getContent().values();
	}

}
