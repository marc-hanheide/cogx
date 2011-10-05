package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionToAct;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;

public class NewIntentionMirror
extends AbstractDialogueComponent {

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(InterpretedIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleInterpretedIntention(_wmc);
					}
		});
	}

	private void handleInterpretedIntention(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					InterpretedIntention intint = getMemoryEntry(addr, InterpretedIntention.class);

					IntentionToAct actint = new IntentionToAct(intint.stringContent, intint.addressContent);

					getLogger().info("adding an IntentionToAct to dialogue WM:\n" + InterpretedUserIntention.baseIntentionToString(actint, ""));
					addToWorkingMemory(newDataID(), actint);
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}
		});
	}

}
