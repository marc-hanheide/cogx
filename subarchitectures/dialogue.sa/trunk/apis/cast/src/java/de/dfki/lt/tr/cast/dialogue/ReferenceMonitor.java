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
import de.dfki.lt.tr.dialogue.discourse.DialogueMoveTranslator;
import de.dfki.lt.tr.dialogue.interpret.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.dialogue.ref.BeliefTranslator;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import eu.cogx.beliefs.slice.GroundedBelief;
import eu.cogx.beliefs.slice.SharedBelief;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Stack;

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
	Stack<DialogueMove> dst = new Stack<DialogueMove>();

	Intention qud = null;
	WorkingMemoryAddress qud_addr = null;

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
		addBeliefChangeFilters(SharedBelief.class);

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(DialogueMove.class,  WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleDialogueMove(_wmc);
					}
				});
/*
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntentionAdd(_wmc);
					}
				});

		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(Intention.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleIntentionDelete(_wmc);
					}
				});
*/
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleCommIntentionAdd(_wmc);
					}
				});

/*
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(CommunicativeIntention.class, WorkingMemoryOperation.DELETE),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleCommIntentionDelete(_wmc);
					}
				});3
 */
}

	private void
	addBeliefChangeFilters(Class cls_) {
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
		bm.remove(_wmc.address);
		triggerRulefileRewrite();
	}

	private void handleDialogueMove(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address);
			DialogueMove dm = (DialogueMove)data.getData();
			dst.push(dm);
			triggerRulefileRewrite();
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

/*
	private void handleIntentionDelete(WorkingMemoryChange _wmc) {
		// somebody deleted it for us
		if (_wmc.address.equals(qud_addr)) {
			log("hmm... the question under discussion has been resolved by somebody else");
			qud_addr = null;
			qud = null;
		}
		triggerRulefileRewrite();
	}

	private void handleIntentionAdd(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address);
			Intention it = (Intention)data.getData();
			log("got an intention!");
			if (BeliefIntentionUtils.isRobotsPrivateIntention(it)) {
				// this is the new QUD
				if (qud != null) {
					log("removing the old question under discussion");
					deleteFromWorkingMemory(qud_addr);
				}
				log("adding new question under discussion");
				qud_addr = _wmc.address;
				qud = it;
				triggerRulefileRewrite();
			}
			else {
				log("ignoring the intention...");
			}
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}
*/

	private void handleCommIntentionDelete(WorkingMemoryChange _wmc) {
		// somebody deleted it for us
		if (_wmc.address.equals(qud_addr)) {
			log("(communicative) hmm... the question under discussion has been resolved by somebody else");  // okay this won't work
//			qud_addr = null;
//			qud = null;
		}
		triggerRulefileRewrite();
	}

	private void handleCommIntentionAdd(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address);
			CommunicativeIntention cit = (CommunicativeIntention)data.getData();
			Intention it = cit.intent;
			log("got an comm. intention!");
			if (BeliefIntentionUtils.isRobotsPrivateIntention(it)) {
				// this is the new QUD
				if (qud != null) {
					log("removing the old question under discussion");
					deleteFromWorkingMemory(qud_addr);
					try {
						deleteFromWorkingMemory(qud.id, getSubarchitectureID());
					}
					catch (SubarchitectureComponentException e) {
						log("hmm, something went wrong while trying to remove the intention: " + e.message);
					}
					qud = null;
					qud_addr = null;
				}
				if (isQuestion(it)) {
					log("adding new question under discussion");
					qud_addr = _wmc.address;
					qud = it;
				}
				else {
					log("this is not a question");
				}
				triggerRulefileRewrite();
			}
			else {
				log("ignoring the intention...");
			}
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	private boolean isQuestion(Intention it) {
		if (it.content.size() > 0) {
			IntentionalContent itc = it.content.get(0);
			if (itc.postconditions instanceof ComplexFormula) {
				ComplexFormula post = (ComplexFormula) itc.postconditions;
				assert (post.op == BinaryOp.conj);
				if (post.forms.size() > 0 && post.forms.get(0) instanceof ModalFormula && ((ModalFormula) post.forms.get(0)).op.equals(IntentionManagementConstants.stateModality)) {
					dFormula sfs = ((ModalFormula) post.forms.get(0)).form;
					if (sfs instanceof ComplexFormula) {
						ComplexFormula xf = (ComplexFormula) sfs;
						assert (xf.op == BinaryOp.conj);
						if (xf.forms.size() > 0) {
							dFormula hohof = xf.forms.get(0);
							if (hohof instanceof ElementaryFormula) {
								return ((ElementaryFormula) hohof).prop.equals("question-answered");
							}
						}
					}
				}
			}
		}
		return false;
	}

/*
	private boolean toPostcondition(Intention it) {
		if (it.content.size() > 0) {
			IntentionalContent itc = it.content.get(0);
			if (itc.postconditions instanceof ComplexFormula) {
				ComplexFormula post = (ComplexFormula) itc.postconditions;
				assert (post.op == BinaryOp.conj);
				if (post.forms.size() > 0 && post.forms.get(0) instanceof ModalFormula && ((ModalFormula) post.forms.get(0)).op.equals(IntentionManagementConstants.stateModality)) {
					dFormula sfs = ((ModalFormula) post.forms.get(0)).form;
					if (sfs instanceof ComplexFormula) {
						ComplexFormula xf = (ComplexFormula) sfs;
						assert (xf.op == BinaryOp.conj);
						if (xf.forms.size() > 0) {
							dFormula hohof = xf.forms.get(0);
							if (hohof instanceof ElementaryFormula) {
								return ((ElementaryFormula) hohof).prop.equals("question-answered");
							}
						}
					}
				}
			}
		}
		return false;
	}
*/

	private void triggerRulefileRewrite() {
		String taskID = newTaskID();
		ProcessingData pd = new ProcessingData(newProcessingDataId());
		m_proposedProcessing.put(taskID, pd);
		String taskGoal = DialogueGoals.BELIEF_MODEL_UPDATE_TASK;
		proposeInformationProcessingTask(taskID, taskGoal);
	}

	// FIXME: this has to be as efficient as possible!
	private void rewriteRulefile(String filename) {
		log("dumping the current belief and discourse model to " + filename);

		BeliefTranslator bTran = new BeliefTranslator();
		for (WorkingMemoryAddress addr : bm.keySet()) {
			dBelief bel = bm.get(addr);
			bTran.addBelief(addr, bel);
		}
		DialogueMoveTranslator dmTran = new DialogueMoveTranslator();
		for (DialogueMove dm : dst) {
			dmTran.addDialogueMove(dm);
		}

		String qud_str = "";
		if (qud != null) {
			String lines[] = BeliefIntentionUtils.intentionToString(qud).split("\n");
			for (String line : Arrays.asList(lines)) {
				qud_str += "% " + line + "\n";
			}
		}
		else {
			qud_str = "% QUD = null\n";
		}

		try {
			BufferedWriter f = new BufferedWriter(new FileWriter(dumpfile));
			f.write(bTran.toRulefileContents());
			f.write(dmTran.toRulefileContents());
			f.write("\n");
			f.write(qud_str);
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
