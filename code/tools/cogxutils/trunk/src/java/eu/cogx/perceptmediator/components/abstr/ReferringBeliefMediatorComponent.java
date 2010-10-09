/**
 * 
 */
package eu.cogx.perceptmediator.components.abstr;


import cast.UnknownSubarchitectureException;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

/**
 * @author marc
 * 
 */
public abstract class ReferringBeliefMediatorComponent<To extends dBelief, Linked extends dBelief> extends
		PerceptMediatorComponent {

	final protected WMView<Linked> allBeliefs;
	final protected Class<Linked> belClass;
	
	/** 
	 * 
	 */
	public ReferringBeliefMediatorComponent(Class<Linked> belClass) {
		this.belClass=belClass;
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
