package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.BaseIntention;
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

									WorkingMemoryAddress wma = iint.addressContent.get("answer-to");
									if (wma != null && wma.equals(lastQUD)) {
										ensureQUDIsNull();
									} 
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
									if (isQUD(actint)) {
										ensureQUDIsNull();
										setQUD(addr);
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
				getLogger().debug("removing the QUD " + wmaToString(lastQUD) + " from the WM");
				deleteFromWorkingMemory(lastQUD);
			}
		}
		catch (SubarchitectureComponentException ex) {
			logException(ex);
		}
		lastQUD = null;
	}

	public boolean isQUD(BaseIntention bint) {
		if (bint.stringContent.get("type").equals("question")) {
			return true;
		}
		if (bint.stringContent.get("type").equals("engagement-opening")) {
			return true;
		}
		if (bint.stringContent.get("type").equals("assertion")) {
			return true;
		}
		return false;
	}

	public void setQUD(WorkingMemoryAddress wma) {
		getLogger().debug("setting QUD to " + wmaToString(wma));
		lastQUD = wma;
	}

}
