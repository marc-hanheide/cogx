/**
 * 
 */
package binder.perceptmediator.components.abstr;

import java.util.Map;

import Ice.ObjectImpl;
import beliefmodels.autogen.beliefs.PerceptBelief;
import binder.perceptmediator.PerceptBindingMediator;
import binder.perceptmediator.components.PlaceConnectivityMediator;
import binder.perceptmediator.components.PlaceMediator;
import binder.perceptmediator.transferfunctions.abstr.DependentDiscreteTransferFunction;
import binder.perceptmediator.transferfunctions.abstr.SimpleDiscreteTransferFunction;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMEntrySynchronizer.TransferFunction;

/**
 * this is an abstract implementation of a general mediator component which
 * takes percepts from working memory and foregrounds them on the binder as
 * {@link PerceptBelief}. It is thought to be specialized for all kinds of
 * percepts. Generally, it employs the {@link PerceptBindingMediator} to monitor
 * any changes in the memory and synchronize them to the binder in a generic
 * way. In order to specialize this class for your specific needs you have to
 * implement an appropriate {@link TransferFunction} (based on
 * {@link SimpleDiscreteTransferFunction} or
 * {@link DependentDiscreteTransferFunction} for instance) and implement the
 * abstract getMediator method.
 * 
 * See {@link PlaceMediator} of {@link PlaceConnectivityMediator} for examples.
 * 
 * @author marc
 * 
 */
public abstract class PerceptMediatorComponent extends ManagedComponent {

	
	/**
	 * the mediator to be used by the component.
	 */
	PerceptBindingMediator<? extends Ice.ObjectImpl> mediator = null;
	
	/**
	 * the mediator runs in a separate thread.
	 */
	Thread mediatorThread = null;

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		// spawn all registered monitors
		mediatorThread = new Thread(mediator, mediator.toString());
		mediatorThread.start();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		mediator = getMediator();
	}

	/** to be implemented to specialize for certain types
	 * @return the {@link PerceptBindingMediator} to be used by this class.
	 */
	protected abstract PerceptBindingMediator<? extends ObjectImpl> getMediator();

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		super.stop();

		mediatorThread.interrupt();
	}

}
