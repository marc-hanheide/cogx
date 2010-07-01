/**
 * 
 */
package eu.cogx.perceptmediator.components.abstr;


import cast.UnknownSubarchitectureException;
import castutils.castextensions.WMView;
import eu.cogx.beliefs.slice.PerceptBelief;

/**
 * @author marc
 * 
 */
public abstract class ReferringPerceptMediatorComponent extends
		PerceptMediatorComponent {

	final protected WMView<PerceptBelief> perceptBeliefsView;

	/**
	 * 
	 */
	public ReferringPerceptMediatorComponent() {
		perceptBeliefsView = WMView.create(this, PerceptBelief.class);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		try {
			perceptBeliefsView.start();
		} catch (UnknownSubarchitectureException e) {
			getLogger().error("trying to start WMView<PerceptBelief>", e);
		}

		super.start();
	}

}
