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

import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.events.Event;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.parseselection.SimpleParseSelection;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parse.PackedLFs;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.dialogue.util.LFUtils;
import java.util.HashMap;
import java.util.Iterator;

public class ParseSelection
extends AbstractDialogueComponent {

	@Override
	public void start() {

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePackedLFs(_wmc);
					}
		});

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PackedLFs.class,  WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handlePackedLFs(_wmc);
					}
		});

	}

	private void handlePackedLFs(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);

			PackedLFs arg = (PackedLFs)data.getData();
			if (arg.finalized == 1) {
				String taskID = newTaskID();
				ProcessingData pd = new ProcessingData(newProcessingDataId());
				pd.add(data);
				m_proposedProcessing.put(taskID, pd);
				String taskGoal = DialogueGoals.PARSESELECTION_TASK;
				proposeInformationProcessingTask(taskID, taskGoal);
			}
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void executeTask(ProcessingData data) throws DialogueException {
		Iterator<CASTData> iter = data.getData();
		if (iter.hasNext()) {
			Object body = iter.next().getData();
			if (body instanceof PackedLFs) {
				PackedLFs plf = (PackedLFs)body;
				LogicalForm lf = SimpleParseSelection.extractLogicalFormWithMood(plf);
				if (lf != null) {
					log("selected the following LF: [" + LFUtils.lfToString(lf) + "]");
					try {
						addToWorkingMemory(newDataID(), lf);
					}
					catch (AlreadyExistsOnWMException ex) {
						ex.printStackTrace();
					}
				}
				else {
					log("no parse found, should generate an event here!");
				}
			}
		}
	}

/*
	public Event createParseErrorEvent() {
		Event e = new Event();
		PrivateEpistemicStatus epst = new PrivateEpistemicStatus();
		epst.agent = "robot";
		e.estatus = epst;
		e.frame = new AbstractFrame();
		CondIndependentDistribs cids = new CondIndependentDistribs();
		cids.distribs = new HashMap<String, ProbDistribution>();
		cids.distribs.put
		return e;
	}
 */
}
