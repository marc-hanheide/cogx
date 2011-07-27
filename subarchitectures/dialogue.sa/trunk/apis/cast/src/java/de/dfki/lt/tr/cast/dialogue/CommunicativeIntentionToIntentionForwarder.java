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
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

public class CommunicativeIntentionToIntentionForwarder
extends AbstractDialogueComponent {

	@Override
	public void start() {
		super.start();

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntention(_wmc);
					}
		});
	}

	private void handleIntention(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);

			CommunicativeIntention cit = (CommunicativeIntention) data.getData();
			Intention it = cit.intent;

			if (it.content.size() == 1 && it.content.get(0).agents.size() == 1 && it.content.get(0).agents.get(0).equals(IntentionManagementConstants.humanAgent) && it.estatus instanceof AttributedEpistemicStatus) {
				String taskID = newTaskID();
				ProcessingData pd = new ProcessingData(newProcessingDataId());
				pd.add(data);
				addProposedTask(taskID, pd);
				String taskGoal = DialogueGoals.INTENTION_MIRRORING_TASK;
				proposeInformationProcessingTask(taskID, taskGoal);
			}
			else {
				log("ignoring an intention");
			}
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void executeTask(ProcessingData data)
	throws DialogueException {
		Iterator<CASTData> iter = data.getData();
		if (iter.hasNext()) {
			Object body = iter.next().getData();

			if (body instanceof CommunicativeIntention) {
				CommunicativeIntention cit = (CommunicativeIntention) body;
				log("forwarding an attributed communicative intention to an intention");

				Intention it = cit.intent;
				try {
					log("adding intention " + it.id + " to dialogue WM:\n" + BeliefIntentionUtils.intentionToString(it));
//					addToWorkingMemory(newDataID(), it);
					addToWorkingMemory(it.id, it);
				}
				catch (Exception e) {
					e.printStackTrace();
					throw new DialogueException(e.getMessage());
				}
			}
		}
		else {
			log("no data for processing");
		}
	}

}
