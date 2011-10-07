package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import java.util.Map;

public class InteractionManager
extends AbstractDialogueComponent {

	private WorkingMemoryAddress lastQUD;

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);
	}

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				InterpretedIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									InterpretedIntention iint = getMemoryEntry(addr, InterpretedIntention.class);
									
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				IntentionToAct.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

							@Override
							public void execute(WorkingMemoryAddress addr) {
								try {
									IntentionToAct actint = getMemoryEntry(addr, IntentionToAct.class);
									if (actint.stringContent.get("type").equals("question")) {
										// it's a QUD!
										ensureQUDIsNull();
										lastQUD = addr;
									}
								}
								catch (SubarchitectureComponentException ex) {
									logException(ex);
								}
							}
							
						});
					}
				});
	}

	public void ensureQUDIsNull() {
		try {
			if (lastQUD != null) {
				deleteFromWorkingMemory(lastQUD);
			}
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
		lastQUD = null;
	}

}
