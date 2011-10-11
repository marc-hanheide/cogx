package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.InterpretedIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.PossibleInterpretedIntentions;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.InterpretedUserIntention;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import java.util.List;
import java.util.Map.Entry;
import java.util.Set;

public class IntentionUnpacker
extends AbstractDialogueComponent {

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PossibleInterpretedIntentions.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePossibleInterpretedIntentions(_wmc);
					}
		});
	}

	private void handlePossibleInterpretedIntentions(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					PossibleInterpretedIntentions pii = getMemoryEntry(addr, PossibleInterpretedIntentions.class);

					WorkingMemoryAddress iintAddr = mostConfidentIntentionAddress(pii);
					InterpretedIntention iint = pii.intentions.get(iintAddr);
					
					for (WorkingMemoryAddress belAddr : iint.addressContent.values()) {
						if (pii.beliefs.containsKey(belAddr)) {
							dBelief bel = pii.beliefs.get(belAddr);
							getLogger().info("adding a dBelief to " + wmaToString(belAddr) + ": " + BeliefIntentionUtils.beliefToString(bel));
							addToWorkingMemory(belAddr, bel);
						}
					}
					getLogger().info("adding an InterpretedIntention to " + wmaToString(iintAddr) + ":\n" + InterpretedUserIntention.interpretedIntentionToString(iint));
					addToWorkingMemory(iintAddr, iint);
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}

		});
	}

	public static WorkingMemoryAddress mostConfidentIntentionAddress(PossibleInterpretedIntentions pii) {
		WorkingMemoryAddress bestAddr = null;
		double best = -1.0;

		for (WorkingMemoryAddress addr : pii.intentions.keySet()) {
			InterpretedIntention iint = pii.intentions.get(addr);
			if (iint.confidence > best) {
				bestAddr = addr;
				best = iint.confidence;
			}
		}
		
		return bestAddr;
	}

}
