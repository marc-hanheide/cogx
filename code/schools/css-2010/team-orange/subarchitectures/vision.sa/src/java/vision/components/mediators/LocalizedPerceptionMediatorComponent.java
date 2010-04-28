/**
 * 
 */
package vision.components.mediators;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import vision.components.mediators.abstr.AbstractLocalizedPerceptionMediator;
import cast.architecture.ManagedComponent;
 
/**
 * @author marc
 * 
 */
public class LocalizedPerceptionMediatorComponent extends ManagedComponent {

	Set<AbstractLocalizedPerceptionMediator<? extends Ice.ObjectImpl,? extends Ice.ObjectImpl>> monitors;
	Set<Thread> monitorThreads;

	/**
	 * 
	 */
	public LocalizedPerceptionMediatorComponent() {
		monitors = new HashSet<AbstractLocalizedPerceptionMediator<? extends Ice.ObjectImpl, ? extends Ice.ObjectImpl>>();
		monitorThreads = new HashSet<Thread>();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();

		// spawn all registered monitors
		log("spawn all monitors");
		for (AbstractLocalizedPerceptionMediator<? extends Ice.ObjectImpl,? extends Ice.ObjectImpl> monitor : monitors) {
			log("spawn monitor " + monitor.getClass().getSimpleName());
			Thread thread = new Thread(monitor);
			thread.start();
			monitorThreads.add(thread);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#configure(java.util.Map)
	 */
	@Override
	protected void configure(Map<String, String> config) {
		monitors.add(new PersonMediator(this));
		monitors.add(new VisualObjectMediator(this));
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#stop()
	 */
	@Override
	protected void stop() {
		super.stop();
		for (Thread t : monitorThreads) {
			t.interrupt();
		}
	}

}
