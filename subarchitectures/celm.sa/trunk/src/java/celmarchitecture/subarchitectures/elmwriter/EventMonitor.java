package celmarchitecture.subarchitectures.elmwriter;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import celm.autogen.CELMEventToStore;
import celm.autogen.CELMPartialEventToStore;
import celmarchitecture.global.GlobalSettings;

/**
 * A simple process which just moves CELMEventToStore and
 * CELMPartialEventToStore objects from other WMs to that of ElmWriter. That way
 * processes in other SAs can also just report events on their own WM.
 * 
 * @author ds
 */
public class EventMonitor extends ManagedComponent {

	private boolean localVerbose = false;
	private boolean verbose = GlobalSettings.verbose || localVerbose;

	public EventMonitor() {

		super();
	}

	@Override
	public void start() {
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				CELMPartialEventToStore.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {

						if (!_wmc.address.subarchitecture
								.equals(getSubarchitectureID()))
							movePartialEvent(_wmc);
						else if (verbose)
							println("found partial event not to move");
					}
				});

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(
				CELMEventToStore.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {

					public void workingMemoryChanged(WorkingMemoryChange _wmc) {

						if (!_wmc.address.subarchitecture
								.equals(getSubarchitectureID()))
							moveEvent(_wmc);
						else if (verbose)
							println("found event not to move");
					}
				});
	}

	private void movePartialEvent(WorkingMemoryChange _ceventChange) {

		try {
			if (verbose)
				println("found partial event to move");

			CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.address);

			CELMPartialEventToStore cevent = (CELMPartialEventToStore) wme
					.getData();

			if (verbose)
				println("about to move");

			addToWorkingMemory(newDataID(), cevent);

			deleteFromWorkingMemory(_ceventChange.address);

			if (verbose)
				println("moved a partial event");

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	private void moveEvent(WorkingMemoryChange _ceventChange) {

		try {
			if (verbose)
				println("found event to move");

			CASTData<?> wme = getWorkingMemoryEntry(_ceventChange.address);

			CELMEventToStore cevent = (CELMEventToStore) wme.getData();

			if (verbose)
				println("about to move");

			addToWorkingMemory(newDataID(), cevent);

			deleteFromWorkingMemory(_ceventChange.address);

			if (verbose)
				println("moved an event");

		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
			if (GlobalSettings.exitOnException)
				System.exit(GlobalSettings.exitValueOnException);
		}
	}

	@Override
	protected void taskAdopted(String _taskID) {
	}

	@Override
	protected void taskRejected(String _taskID) {
	}

}
