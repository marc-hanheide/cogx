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

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import java.util.Stack;

import cast.SubarchitectureComponentException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.FilterRestriction;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import de.dfki.lt.tr.beliefs.slice.intentions.CommunicativeIntention;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.discourse.DialogueMoveTranslator;
import de.dfki.lt.tr.dialogue.util.BeliefIntentionUtils;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import de.dfki.lt.tr.dialogue.ref.impl.abductive.BeliefTranslator;
import de.dfki.lt.tr.dialogue.slice.discourse.DialogueMove;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.List;

/**
 * 
 * @author Miroslav Janicek
 */
public class ContextMonitor
extends AbstractDialogueComponent {

	private final String DEFAULT_DUMPFILE = "/dev/null";
	private final String DEFAULT_BELIEF_LISTEN_SA = "";

	private String dumpfile;
	private String beliefListenSA;

	private Map<WorkingMemoryAddress, dBelief> bm;
	private Stack<DialogueMove> dst;

	private Intention qud;

	public ContextMonitor() {
		dumpfile = DEFAULT_DUMPFILE;
		beliefListenSA = DEFAULT_BELIEF_LISTEN_SA;
		
		bm = new HashMap<WorkingMemoryAddress, dBelief>();
		dst = new Stack<DialogueMove>();
		
		qud = null;
	}

	@Override
	public void onConfigure(Map<String, String> args) {
		super.onConfigure(args);

		if (args.containsKey("--dumpfile")) {
			dumpfile = args.get("--dumpfile");
		}
		if (args.containsKey("--belief-listen-SA")) {
			beliefListenSA = args.get("--belief-listen-SA");
		}
	}

	@Override
	public void onStart() {
		super.onStart();
		triggerRulefileRewrite();

		addBeliefChangeFilters(dBelief.class);

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				DialogueMove.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleDialogueMove(_wmc);
					}
				});

		addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
				CommunicativeIntention.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					@Override
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleCommIntentionAdd(_wmc);
					}
				});
	}

	private void addBeliefChangeFilters(Class cls_) {
		addChangeFilter(ChangeFilterFactory.createChangeFilter(cls_,
				WorkingMemoryOperation.ADD, "", "", beliefListenSA,
				FilterRestriction.ALLSA), new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				handleBeliefAddOverwrite(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createChangeFilter(cls_,
				WorkingMemoryOperation.OVERWRITE, "", "", beliefListenSA,
				FilterRestriction.ALLSA), new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				handleBeliefAddOverwrite(_wmc);
			}
		});

		addChangeFilter(ChangeFilterFactory.createChangeFilter(cls_,
				WorkingMemoryOperation.DELETE, "", "", beliefListenSA,
				FilterRestriction.ALLSA), new WorkingMemoryChangeReceiver() {
			@Override
			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				handleBeliefDelete(_wmc);
			}
		});
	}

	private void handleBeliefAddOverwrite(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					dBelief bel = getMemoryEntry(addr, dBelief.class);
					bm.put(addr, bel);
					triggerRulefileRewrite();
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("exception in belief add/overwrite", ex);
				}
			}
		
		});
	}

	private void handleBeliefDelete(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				bm.remove(addr);
				triggerRulefileRewrite();
			}
			
		});
	}

	private void handleDialogueMove(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					DialogueMove dm = getMemoryEntry(addr, DialogueMove.class);
					dst.push(dm);
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("exception in handling dialogue move", ex);
				}
			}
		});
	}

	private void handleCommIntentionAdd(WorkingMemoryChange _wmc) {
		addTask(new ProcessingTaskWithData<WorkingMemoryAddress>(_wmc.address) {

			@Override
			public void execute(WorkingMemoryAddress addr) {
				try {
					CommunicativeIntention cit = getMemoryEntry(addr, CommunicativeIntention.class);
					Intention it = cit.intent;
					log("got a communicative intention");

					// HACK
					// deleting previous intention here, this signals to the planner
					// that the action has been finished (e.g. that the dialogue has
					// been completed)

					// robot's communicative intention
					if (BeliefIntentionUtils.isRobotsIntention(it)) {
						// the robot wants to communicate: only remove the QUD if
						// the new intention is a question
						if (isQuestion(it)) {
							removeQUD();
							log("setting the QUD");
							qud = it;
							triggerRulefileRewrite();
						}
						else {
							log("the robot wants to talk, but does not ask a question -> the QUD stays");
						}
					}
					else if (BeliefIntentionUtils.isHumansIntention(it)) {
						log("got the human's intention, assuming that it resolves/overrides the QUD");
						removeQUD();
						triggerRulefileRewrite();
					}
					else {
						log("ignoring the intention");
					}
				}
				catch (SubarchitectureComponentException ex) {
					getLogger().error("component exception", ex);
				}
			}

		});

	}

	private void removeQUD() {
		if (qud != null) {
			try {
				log("deleting qud.intent [" + qud.id + "]");
				deleteFromWorkingMemory(qud.id);
			}
			catch (SubarchitectureComponentException ex) {
				getLogger().error("problem when deleting input intention", ex);
			}
			qud = null;
		}
		else {
			log("no QUD to remove");
		}
	}

	private static boolean isQuestion(Intention it) {
		if (it.content.size() > 0) {
			IntentionalContent itc = it.content.get(0);
			if (itc.postconditions instanceof ComplexFormula) {
				ComplexFormula post = (ComplexFormula) itc.postconditions;
				assert (post.op == BinaryOp.conj);
				if (post.forms.size() > 0
						&& post.forms.get(0) instanceof ModalFormula
						&& ((ModalFormula) post.forms.get(0)).op
								.equals(IntentionManagementConstants.stateModality)) {
					dFormula sfs = ((ModalFormula) post.forms.get(0)).form;
					if (sfs instanceof ComplexFormula) {
						ComplexFormula xf = (ComplexFormula) sfs;
						assert (xf.op == BinaryOp.conj);
						if (xf.forms.size() > 0) {
							dFormula hohof = xf.forms.get(0);
							if (hohof instanceof ElementaryFormula) {
								return ((ElementaryFormula) hohof).prop
										.equals("question-answered");
							}
						}
					}
				}
			}
		}
		return false;
	}

	private void triggerRulefileRewrite() {
		addTask(new ProcessingTaskWithoutData() {

			@Override
			public void execute() {
				rewriteRulefile(dumpfile);
			}
			
		});
	}

	// NOTE: this has to be as efficient as possible!
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

		String qud_str = "% question under discussion\n";
		if (qud != null) {
			List<ModalisedAtom> qud_facts = ConversionUtils.intentionToFacts(TermAtomFactory.term("qud"), qud);
			for (ModalisedAtom fact : qud_facts) {
				qud_str += PrettyPrint.modalisedAtomToString(fact) + ".\n";
			}
		} else {
			qud_str += "% (none)\n";
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
			getLogger().error("I/O error while writing the dumpfile", ex);
		}
	}

}
