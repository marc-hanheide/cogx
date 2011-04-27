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
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.interpret.BeliefFormulaFactory;
import de.dfki.lt.tr.dialogue.interpret.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import eu.cogx.beliefs.slice.SharedBelief;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

/**
 *
 * @author Miroslav Janicek
 */
public class NewObjectMonitor extends AbstractDialogueComponent {

	private static final String LISTEN_SUBARCH = "binder";

	@Override
	public void start() {
		super.start();
		addChangeFilter(ChangeFilterFactory.createChangeFilter(SharedBelief.class,
				WorkingMemoryOperation.ADD, "", "", LISTEN_SUBARCH,
				FilterRestriction.ALLSA), new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				handleBeliefAdd(_wmc);
			}
		});
	}

	private void handleBeliefAdd(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address);
			String taskID = newTaskID();
			ProcessingData pd = new ProcessingData(newProcessingDataId());
			pd.add(data);
			m_proposedProcessing.put(taskID, pd);
			String taskGoal = DialogueGoals.BELIEF_MODEL_UPDATE_TASK;  // FIXME: task name wrong
			proposeInformationProcessingTask(taskID, taskGoal);
			log("observed a new object!");
		} catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}


	@Override
	public void executeTask(ProcessingData data) throws DialogueException {
		Iterator<CASTData> iter = data.getData();
		if (iter.hasNext()) {
			CASTData d = iter.next();
			Object body = d.getData();
			if (body instanceof dBelief) {
				dBelief b = (dBelief) body;
				this.sleepComponent(100);
				log("announcing the new object");

				Intention it = new Intention();
				it.id = newDataID();
				it.estatus = new PrivateEpistemicStatus(IntentionManagementConstants.thisAgent);
				it.content = new LinkedList<IntentionalContent>();

				List<String> ags = new LinkedList<String>();
				ags.add(IntentionManagementConstants.thisAgent);

				ComplexFormula inState = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
				inState.forms.add(new ElementaryFormula(0, "new-object-announced"));
				inState.forms.add(new ModalFormula(0, "about-shared", BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(b.id, LISTEN_SUBARCH))));

				ModalFormula state = new ModalFormula(0, IntentionManagementConstants.stateModality, inState);

				ComplexFormula post = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
				post.forms.add(state);

				IntentionalContent itc = new IntentionalContent(ags, new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj), post, 1.0f);
				it.content.add(itc);
				it.frame = new AbstractFrame();

//				CommunicativeIntention cit = new CommunicativeIntention(it);

				log("writing intention " + it.id + " to dialogue WM:\n" + BeliefIntentionUtils.intentionToString(it));
				try {
					addToWorkingMemory(it.id, it);
				} catch (AlreadyExistsOnWMException ex) {
					ex.printStackTrace();
				}
			}
		}
	}

}