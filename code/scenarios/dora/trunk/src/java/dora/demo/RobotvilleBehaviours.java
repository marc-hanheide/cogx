package dora.demo;

import SpatialData.Place;
import spatial.demo.PlacePatroller;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

public class RobotvilleBehaviours extends ManagedComponent {

	private final PlacePatroller m_patroller;
	private final Thread m_patrollerThread;

	public RobotvilleBehaviours() {
		m_patroller = new PlacePatroller(this);
		m_patrollerThread = new Thread(m_patroller);
		m_patrollerThread.start();
	}

	@Override
	protected void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Place.class, WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
			
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc)
					throws CASTException {
				println("adding place");
				m_patroller.addPlace(getMemoryEntry(_wmc.address, Place.class));				
			}
		});
	}

	@Override
	protected void stop() {
		m_patroller.kill();
		// TODO what instead?
		// m_patrollerThread.stop();
	}

	@Override
	protected void runComponent() {
		// sleep for a while to get started
		sleepComponent(30000);
		while (isRunning()) {
			println("starting patrolling");
			m_patroller.startPatrolling();
			sleepComponent(30000);
			println("stoppiing patrolling");
			m_patroller.stopPatrolling();
		}

	}

}
