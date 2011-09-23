package de.dfki.lt.tr.cast.dialogue;

import de.dfki.lt.tr.dialogue.ref.impl.discourse.DiscourseResolver;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;

public class DiscourseReferenceResolution
extends AbstractReferenceResolutionComponent<DiscourseResolver> {

	public DiscourseReferenceResolution() {
		super(new DiscourseResolver());
	}

	@Override
	protected void onStart() {
		super.onStart();
		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				DialogueMove.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleAddDialogueTask(_wmc);
					}
				});
	}

	protected void handleAddDialogueTask(WorkingMemoryChange wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					DialogueMove dm = getMemoryEntry(addr, DialogueMove.class);
					getResolver().addDialogueMove(dm);
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("exception in handling a new dialogue move", ex);
				}
			}

		});
	}

}
