package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.slice.StandbyMode;
import de.dfki.lt.tr.dialogue.slice.asr.InitialNoise;
import de.dfki.lt.tr.dialogue.slice.asr.InitialPhonString;
import de.dfki.lt.tr.dialogue.slice.asr.UnclarifiedPhonString;

public abstract class AbstractStandbyModeManager
extends AbstractDialogueComponent {

	private WorkingMemoryAddress current;

	public AbstractStandbyModeManager() {
		super();
		current = null;
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(StandbyMode.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {
							@Override
							public void execute(WorkingMemoryAddress addr) {
								getLogger().info("StandbyMode ADD -> turning on local standby mode");
								current = addr;
								ProcessingTask ptask = onStandbyOn();
								if (ptask != null) {
									addTask(ptask);
								}
							}
						});
					}
				});
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(StandbyMode.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithoutData() {
							@Override
							public void execute() {
								log("StandbyMode DELETE -> turning off local standby mode");
								current = null;
								ProcessingTask ptask = onStandbyOff();
								if (ptask != null) {
									addTask(ptask);
								}
							}
						});
					}
				});
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(InitialPhonString.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {
							@Override
							public void execute(WorkingMemoryAddress addr) {
								handleInitialPhonString(addr);
							}
						});
					}
				});
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(InitialNoise.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {
							@Override
							public void execute(WorkingMemoryAddress addr) {
								handleInitialNoise(addr);
							}
						});
					}
				});
	}

	private boolean onStandby() {
		return current != null;
	}

	private void handleInitialPhonString(WorkingMemoryAddress addr) {
		try {
			InitialPhonString ips = getMemoryEntry(addr, InitialPhonString.class);

			if (onStandby()) {
				log("ignoring an InitialPhonString (in standby mode)");
			}
			else {
				log("forwarding an UnclarifiedPhonString");
				addToWorkingMemory(ips.ps.id, new UnclarifiedPhonString(ips.ps));
			}
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	private void handleInitialNoise(WorkingMemoryAddress addr) {
		try {
			InitialNoise in = getMemoryEntry(addr, InitialNoise.class);
			if (onStandby()) {
				log("ignoring an InitialNoise (in standby mode)");
			}
			else {
				log("forwarding a Noise");
				addToWorkingMemory(newDataID(), in.n);
			}
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
	}

	public abstract ProcessingTask onStandbyOn();

	public abstract ProcessingTask onStandbyOff();

}
