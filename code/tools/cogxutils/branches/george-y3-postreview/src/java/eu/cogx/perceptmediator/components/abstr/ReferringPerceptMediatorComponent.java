/**
 * 
 */
package eu.cogx.perceptmediator.components.abstr;

import Ice.ObjectImpl;
import cast.UnknownSubarchitectureException;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import eu.cogx.perceptmediator.PerceptBindingMediator;

/**
 * @author marc
 * 
 */
public abstract class ReferringPerceptMediatorComponent<To extends dBelief>
		extends PerceptMediatorComponent {

	final protected WMView<To> allBeliefs;
	final protected Class<To> belClass;

	/**
	 * 
	 */
	public ReferringPerceptMediatorComponent(Class<To> belClass) {
		this.belClass = belClass;
		allBeliefs = WMView.create(this, belClass);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		try {
			allBeliefs.start();
		} catch (UnknownSubarchitectureException e) {
			getLogger().error("trying to start WMView<To>", e);
		}

		super.start();
	}

}
