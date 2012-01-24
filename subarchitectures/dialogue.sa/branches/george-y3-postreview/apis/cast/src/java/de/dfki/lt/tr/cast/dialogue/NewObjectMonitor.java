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
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.util.BeliefFormulaFactory;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 *
 * @author Miroslav Janicek
 */
public class NewObjectMonitor
extends AbstractDialogueComponent {

	private String listenSA;

	public NewObjectMonitor() {
		listenSA = null;
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);
		if (args.containsKey("--listen-sa")) {
			listenSA = args.get("--listen-sa");
		}
	}

	@Override
	public void onStart() {
		super.onStart();
		if (listenSA != null) {
			addChangeFilter(ChangeFilterFactory.createChangeFilter(dBelief.class,
					WorkingMemoryOperation.ADD, "", "", listenSA,
					FilterRestriction.ALLSA), new WorkingMemoryChangeReceiver() {
				@Override
				public void workingMemoryChanged(WorkingMemoryChange _wmc) {
					handleBeliefAdd(_wmc);
				}
			});
		}
		else {
			getLogger().warn("don't know what to listen for");
			scheduleOwnDeath();
		}
	}

	private void handleBeliefAdd(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					dBelief bel = getMemoryEntry(addr, dBelief.class);

					sleepComponent(100);
					log("announcing the new object: [" + addr.id + "," + addr.subarchitecture + "]");

					Intention it = new Intention();
					it.id = newDataID();
					it.estatus = new PrivateEpistemicStatus(IntentionManagementConstants.thisAgent);
					it.content = new LinkedList<IntentionalContent>();

					List<String> ags = new LinkedList<String>();
					ags.add(IntentionManagementConstants.thisAgent);

					ComplexFormula inState = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
					inState.forms.add(new ElementaryFormula(0, "new-object-announced"));
					inState.forms.add(new ModalFormula(0, "about-shared", BeliefFormulaFactory.newPointerFormula(addr)));

					ModalFormula state = new ModalFormula(0, IntentionManagementConstants.stateModality, inState);

					ComplexFormula post = new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj);
					post.forms.add(state);

					IntentionalContent itc = new IntentionalContent(ags, new ComplexFormula(0, new LinkedList<dFormula>(), BinaryOp.conj), post, 1.0f);
					it.content.add(itc);
					it.frame = new AbstractFrame();

	//				CommunicativeIntention cit = new CommunicativeIntention(it);

					log("writing intention " + it.id + " to dialogue WM:\n" + BeliefIntentionUtils.intentionToString(it));
					addToWorkingMemory(it.id, it);

				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("subarch component exception", ex);
				}
			}
			
		});
	}

}
