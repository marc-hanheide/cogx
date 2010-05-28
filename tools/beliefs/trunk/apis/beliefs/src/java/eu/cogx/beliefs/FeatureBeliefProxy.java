package eu.cogx.beliefs;

import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

public class FeatureBeliefProxy extends BeliefProxy {

	/**
	 * @param id
	 */
	public FeatureBeliefProxy(String id) {
		super(id);
		this._belief.content=new ConditionallyIndependentDistributionProxy().getContent();
	}

	public FeatureBeliefProxy(dBelief belief) {
		super(belief);
		assert(belief.content instanceof CondIndependentDistribs);
	}

	/* (non-Javadoc)
	 * @see eu.cogx.beliefs.BeliefDelegate#getContent()
	 */
	@Override
	public IndependentFeatureDistributionProxy getContent() {
		return new IndependentFeatureDistributionProxy((CondIndependentDistribs) _belief.content);
	}

	/* (non-Javadoc)
	 * @see eu.cogx.beliefs.BeliefDelegate#setContent(eu.cogx.beliefs.ContentDelegate)
	 */
	public void setContent(
			IndependentFeatureDistributionProxy conditionallyIndependentDistributionDelegate)
			throws BeliefNotInitializedException, BeliefMissingValueException {
		super.setContent(conditionallyIndependentDistributionDelegate);
	}

	
	
}
