/**
 * 
 */
package binder.components.perceptmediator;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import binder.components.perceptmediator.transferfunctions.ConnectivityTransferFunction;
import binder.components.perceptmediator.transferfunctions.PlaceTransferFunction;

import SpatialData.Place;
import SpatialProperties.ConnectivityPathProperty;

import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import castutils.castextensions.WMView;

/**
 * @author marc
 * 
 */
public class PerceptMediatorComponent extends ManagedComponent {

	Set<PerceptMonitor<? extends Ice.ObjectImpl>> monitors;
	Set<Thread> monitorThreads;
	private WMView<Place> places;

	/**
	 * 
	 */
	public PerceptMediatorComponent() {
		monitors = new HashSet<PerceptMonitor<? extends Ice.ObjectImpl>>();
		monitorThreads = new HashSet<Thread>();
		places=WMView.create(this, Place.class);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.core.CASTComponent#start()
	 */
	@Override
	protected void start() {
		super.start();
		try {
			places.start();
		} catch (UnknownSubarchitectureException e) {
			getLogger().error("trying to start WMView<Place>", e);
		}
		// spawn all registered monitors
		log("spawn all monitors");
		for (PerceptMonitor<? extends Ice.ObjectImpl> monitor : monitors) {
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
		monitors.add(new PerceptMonitor<Place>(this, Place.class,
				new PlaceTransferFunction()));
		monitors.add(new PerceptMonitor<ConnectivityPathProperty>(this,
				ConnectivityPathProperty.class,
				new ConnectivityTransferFunction(places, PerceptBeliefManager.getManager(this))));
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
