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
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;

public class CommunicativeIntentionToIntentionForwarder
extends AbstractDialogueComponent {

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntention(_wmc);
					}
				});
	}

	private void handleIntention(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryChange>(_wmc) {

			@Override
			public void execute(WorkingMemoryChange wmc) {
				try {
					CommunicativeIntention cit = getMemoryEntry(wmc.address, CommunicativeIntention.class);
					Intention it = cit.intent;

					if (it.content.size() == 1
							&& it.content.get(0).agents.size() == 1
							&& it.content.get(0).agents.get(0).equals(IntentionManagementConstants.humanAgent)
							&& it.estatus instanceof AttributedEpistemicStatus) {

						log("forwarding an attributed communicative intention to an intention");
						log("adding intention " + it.id + " to dialogue WM:\n" + BeliefIntentionUtils.intentionToString(it));
						addToWorkingMemory(it.id, it);
					}
					else {
						getLogger().debug("ignoring an intention that is not the human's");
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarchitecture component exception", ex);
				}
			}
			
		});
	}

}
