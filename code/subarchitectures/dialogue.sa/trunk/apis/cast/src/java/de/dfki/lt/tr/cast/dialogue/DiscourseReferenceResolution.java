package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
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
						addTask(newAddDialogueTask(_wmc));
					}
				});
	}

	protected ProcessingTaskWithData<WorkingMemoryChange> newAddDialogueTask(WorkingMemoryChange wmc) {
		return new ProcessingTaskWithData<WorkingMemoryChange>(wmc) {

			@Override
			public void execute(WorkingMemoryChange wmc) {
				try {
					DialogueMove dm = getMemoryEntry(wmc.address, DialogueMove.class);
					getResolver().addDialogueMove(dm);
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("exception in handling a new dialogue move", ex);
				}
			}

		};
	}

}
