package spatial.demo;

import java.util.Iterator;

import spatial.execution.SpatialActionInterface;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.PlaceStatus;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.CASTHelper;

public class PlacePatroller extends CASTHelper implements Runnable,
		WorkingMemoryChangeReceiver {

	private WorkingMemoryAddress m_navCmdAddr;
	private Object m_patrolSignal = new Object();
	private boolean m_patrol;
	private boolean m_kill;
	private final PatrolSchedule m_schedule;
	private Iterator<Place> m_placeInterator;

	public PlacePatroller(ManagedComponent _c) {
		super(_c);
		m_schedule = new AdditionOrderPatrolSchedule();
		m_patrol = false;
		m_kill = false;
	}

	public void addPlace(Place _p) {
		if (_p.status == PlaceStatus.TRUEPLACE)
			m_schedule.addPlace(_p);
	}

	private void goToPlace(Place _p) throws AlreadyExistsOnWMException,
			DoesNotExistOnWMException, UnknownSubarchitectureException {

		assert (m_navCmdAddr == null);

		NavCommand navCommand = SpatialActionInterface.newNavCommand();
		navCommand.cmd = CommandType.GOTOPLACE;
		navCommand.destId = new long[] { _p.id };

		log("going to place: " + _p.id);
		
		m_navCmdAddr = new WorkingMemoryAddress(component.newDataID(),
				component.getSubarchitectureID());

		component.addChangeFilter(ChangeFilterFactory.createAddressFilter(
				m_navCmdAddr, WorkingMemoryOperation.OVERWRITE), this);

		component.addToWorkingMemory(m_navCmdAddr, navCommand);
	}

	public void startPatrolling() {
		m_patrol = true;
		initialisePatrolSchedule();
		synchronized (m_patrolSignal) {
			m_patrolSignal.notify();
		}
		log("starting patrol");
	}

	/**
	 * 
	 */
	private void initialisePatrolSchedule() {
		m_placeInterator = m_schedule.iterator();
	}

	public void stopPatrolling() {
		m_patrol = false;
	}

	public void kill() {
		m_kill = true;
		synchronized (m_patrolSignal) {
			m_patrolSignal.notify();
		}
		
	}

	private boolean isExecutingCommand() {
		return m_navCmdAddr != null;
	}

	@Override
	public void run() {
		
		while (!m_kill) {

			log("loop");
			try {
				// if patrolling is active
				if (m_patrol) {

					log("in patrol");
					// if we're not executing a command
					if (!isExecutingCommand()) {

						if (!m_placeInterator.hasNext()) {
							initialisePatrolSchedule();
						}

						log("going to place");
						goToPlace(m_placeInterator.next());

					}
					// else we are, just wait for events
					else {
						log("waiting on changes");
						
						component.waitForChanges();
					}

				} else {
					log("not in patrol");
					
					synchronized (m_patrolSignal) {
						try {
							m_patrolSignal.wait();
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
				}
			} catch (CASTException e) {
				component.logException(e);
			}

		}

	}

	private boolean commandCompleted(NavCommand _cmd) {
		if (_cmd.comp == Completion.COMMANDABORTED
				|| _cmd.comp == Completion.COMMANDFAILED
				|| _cmd.comp == Completion.COMMANDSUCCEEDED) {
			return true;
		} else {
			return false;
		}
	}

	@Override
	public void workingMemoryChanged(WorkingMemoryChange _wmc)
			throws CASTException {

		assert (_wmc.address.equals(m_navCmdAddr));
		NavCommand navCommand = component.getMemoryEntry(_wmc.address,
				NavCommand.class);
		if (commandCompleted(navCommand)) {
			log("Command completed");
			component.deleteFromWorkingMemory(m_navCmdAddr);
			m_navCmdAddr = null;
			component.removeChangeFilter(this);
		}

	}
}
