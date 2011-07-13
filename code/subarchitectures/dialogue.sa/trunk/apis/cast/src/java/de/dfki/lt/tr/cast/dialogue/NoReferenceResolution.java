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
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.slice.parseselection.SelectedLogicalForm;
import de.dfki.lt.tr.dialogue.slice.ref.NominalEpistemicReferenceHypothesis;
import de.dfki.lt.tr.dialogue.slice.ref.RefLogicalForm;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import java.util.Iterator;
import java.util.LinkedList;

public class NoReferenceResolution
extends AbstractDialogueComponent {

	@Override
	public void start() {
		super.start();

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(SelectedLogicalForm.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleLogicalForm(_wmc);
					}
				});
	}

	private void handleLogicalForm(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);
			SelectedLogicalForm arg = (SelectedLogicalForm)data.getData();
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = DialogueGoals.REFERENCE_RESOLUTION_TASK;
			proposeInformationProcessingTask(taskID, taskGoal);
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

			if (body instanceof SelectedLogicalForm) {
				SelectedLogicalForm slf = (SelectedLogicalForm) body;
				RefLogicalForm rlf = new RefLogicalForm(slf.lform, slf.ival, new LinkedList<NominalEpistemicReferenceHypothesis>());

				log("adding a RefLogicalForm to the working memory");
				try {
					addToWorkingMemory(newDataID(), rlf);
				}
				catch (AlreadyExistsOnWMException ex) {
					ex.printStackTrace();
				}
			}
		}
		else {
			log("no data for processing");
		}
	}

}
