/**
 * 
 */
package eu.cogx.beliefs;

import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;

/**
 * @author marc
 *
 */
public class IndependentFeatureDistributionProxy extends
		ConditionallyIndependentDistributionProxy {

	/**
	 * 
	 */
	public IndependentFeatureDistributionProxy() {
		super();
	}

	/**
	 * @param pd
	 */
	public IndependentFeatureDistributionProxy(CondIndependentDistribs pd) {
		super(pd);
	}

	/* (non-Javadoc)
	 * @see eu.cogx.beliefs.ConditionallyIndependentDistributionDelegate#get(java.lang.Object)
	 */
	@Override
	public BasicProbDistributionProxy get(Object arg0) {
		return (BasicProbDistributionProxy) super.get(arg0);
	}

}
