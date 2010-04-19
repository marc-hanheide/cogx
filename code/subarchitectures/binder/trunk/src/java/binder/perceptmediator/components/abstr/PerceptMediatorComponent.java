/**
 * 
 */
package binder.perceptmediator.components.abstr;

import java.util.Map;

import Ice.ObjectImpl;
import binder.perceptmediator.PerceptBindingMediator;
import cast.architecture.ManagedComponent;

/**
 * @author marc
 * 
 */
public abstract class PerceptMediatorComponent extends ManagedComponent {

	PerceptBindingMediator<? extends Ice.ObjectImpl> mediator = null;
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
