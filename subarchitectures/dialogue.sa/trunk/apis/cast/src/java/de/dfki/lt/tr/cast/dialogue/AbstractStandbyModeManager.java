package de.dfki.lt.tr.cast.dialogue;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.slice.StandbyMode;

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
	}

	public abstract ProcessingTask onStandbyOn();

	public abstract ProcessingTask onStandbyOff();

}
