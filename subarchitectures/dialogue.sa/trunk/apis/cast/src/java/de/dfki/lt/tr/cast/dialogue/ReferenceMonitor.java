// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots
// Miroslav Janicek (miroslav.janicek@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

package de.dfki.lt.tr.cast.dialogue;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.ref.BeliefTranslator;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.SharedBelief;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

/**
 *
 * @author Miroslav Janicek
 */
public class ReferenceMonitor
extends AbstractDialogueComponent {

//	private AbductiveReferenceResolution arr;
	private String dumpfile = "/dev/null";
	private boolean testing = false;

	HashMap<WorkingMemoryAddress, dBelief> bm = new HashMap<WorkingMemoryAddress, dBelief>();

	@Override
	public void start() {
		super.start();
//		arr = new AbductiveReferenceResolution();

		rewriteRulefile(dumpfile);

/*
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleBeliefAddOverwrite(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleBeliefAddOverwrite(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(dBelief.class,  WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleBeliefDelete(_wmc);
					}
				});

 */
		if (testing) {
			addChangeFilters(dBelief.class);
		}
		else {
			addChangeFilters(GroundedBelief.class);
//			addChangeFilters(SharedBelief.class);
		}
}

	private void
	addChangeFilters(Class cls_) {
		addChangeFilter(
				ChangeFilterFactory.createChangeFilter(cls_, WorkingMemoryOperation.ADD, "", "", "binder", FilterRestriction.ALLSA),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleBeliefAddOverwrite(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createChangeFilter(cls_, WorkingMemoryOperation.OVERWRITE, "", "", "binder", FilterRestriction.ALLSA),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleBeliefAddOverwrite(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createChangeFilter(cls_, WorkingMemoryOperation.DELETE, "", "", "binder", FilterRestriction.ALLSA),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleBeliefDelete(_wmc);
					}
				});
	}

	@Override
	public void configure(Map<String, String> _config)
	{
		if (_config.containsKey("--dumpfile")) {
			dumpfile = _config.get("--dumpfile");
		}
		if (_config.containsKey("--testing")) {
			testing = true;
		}
	}

	private void handleBeliefAddOverwrite(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address);
			dBelief belief = (dBelief)data.getData();
			bm.put(_wmc.address, belief);
			triggerRulefileRewrite();
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	private void handleBeliefDelete(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address);
			dBelief belief = (dBelief)data.getData();
			bm.remove(_wmc.address);
			triggerRulefileRewrite();
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	private void triggerRulefileRewrite() {
		String taskID = newTaskID();
		ProcessingData pd = new ProcessingData(newProcessingDataId());
		m_proposedProcessing.put(taskID, pd);
		String taskGoal = DialogueGoals.BELIEF_MODEL_UPDATE_TASK;
		proposeInformationProcessingTask(taskID, taskGoal);
	}

	// FIXME: this has to be as efficient as possible!
	private void rewriteRulefile(String filename) {
		log("dumping the current belief model to " + filename);

		BeliefTranslator tran = new BeliefTranslator();

		for (WorkingMemoryAddress addr : bm.keySet()) {
			dBelief bel = bm.get(addr);
			tran.addBelief(addr, bel);
		}

		try {
			BufferedWriter f = new BufferedWriter(new FileWriter(dumpfile));
			f.write(tran.toRulefileContents());
			f.close();
		}
		catch (IOException ex) {
			log("I/O error while writing the dumpfile");
		}
	}

	@Override
	public void executeTask(ProcessingData data) throws DialogueException {
		// don't care about the data really
		rewriteRulefile(dumpfile);
	}

}
