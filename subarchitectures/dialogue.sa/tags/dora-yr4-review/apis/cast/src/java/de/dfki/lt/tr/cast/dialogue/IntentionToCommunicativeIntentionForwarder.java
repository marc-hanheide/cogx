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
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;

public class IntentionToCommunicativeIntentionForwarder
extends AbstractDialogueComponent {

	@Override
	public void onStart() {
		super.onStart();

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntention(_wmc);
					}
		});
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class, WorkingMemoryOperation.OVERWRITE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntention(_wmc);
					}
		});
	}

	private void handleIntention(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					Intention it = getMemoryEntry(addr, Intention.class);

					if (it.content.size() == 1 && it.content.get(0).agents.size() == 1
							&& (
								(it.content.get(0).agents.get(0).equals(IntentionManagementConstants.thisAgent)
									&& it.estatus instanceof PrivateEpistemicStatus)
								||
								(it.content.get(0).agents.get(0).equals(IntentionManagementConstants.thisAgent)
									&& it.estatus instanceof SharedEpistemicStatus)
							   )) {

						getLogger().info("forwarding a private intention to communicative intention");

						// set private epistemic status
						CommunicativeIntention cit = new CommunicativeIntention();
						cit.intent = it;
						log("adding communicative intention for " + it.id + " to dialogue WM:\n" + BeliefIntentionUtils.intentionToString(it));
						addToWorkingMemory(newDataID(), cit);
					}
					else {
						getLogger().debug("ignoring an intention that is not the robot's private");
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}
		});
	}

}
