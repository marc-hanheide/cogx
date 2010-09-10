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
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.interpret.ConversionUtils;
import de.dfki.lt.tr.dialogue.ref.AbductiveReferenceResolution;
import de.dfki.lt.tr.dialogue.util.DialogueException;
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 *
 * @author Miroslav Janicek
 */
public class ReferenceMonitor
extends AbstractDialogueComponent {

//	private AbductiveReferenceResolution arr;
	private String dumpfile = "/dev/null";

	HashMap<String, dBelief> bm = new HashMap<String, dBelief>();

	@Override
	public void start() {
		super.start();
//		arr = new AbductiveReferenceResolution();

		rewriteRulefile(dumpfile);

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
	}

	@Override
	public void configure(Map<String, String> _config)
	{
		if (_config.containsKey("--dumpfile")) {
			dumpfile = _config.get("--dumpfile");
		}
	}

	private void handleBeliefAddOverwrite(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);
			dBelief belief = (dBelief)data.getData();
			bm.put(_wmc.address.id, belief);
			triggerRulefileRewrite();
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
	}

	private void handleBeliefDelete(WorkingMemoryChange _wmc) {
		try {
			CASTData data = getWorkingMemoryEntry(_wmc.address.id);
			dBelief belief = (dBelief)data.getData();
			bm.remove(_wmc.address.id);
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

		StringBuilder sb = new StringBuilder();

		// populate epistemic statuses
		List<String> args = new LinkedList<String>();
		for (dBelief bel : bm.values()) {
			args.add("bel : " + MercuryUtils.atomToString(TermAtomFactory.atom("epistemic_status",
						new Term[] {
							TermAtomFactory.term(bel.id),
							ConversionUtils.epistemicStatusToTerm(bel.estatus)
						} ))
					+ ".");
		}
		sb.append(join("\n", args)).append("\n");

		sb.append("\n");

		// populate belief_exist
		args.clear();
		for (dBelief bel : bm.values()) {
			args.add("(bel : b('" + bel.id + "')) = p(1.0)");  // FIXME: how to get belief existence prob?
		}
		sb.append("belief_exist = [\n\t").append(join(",\n\t", args)).append("\n].\n");

		sb.append("\n");

		// populate world_exist & disjoint declarations
		args.clear();
		List<String> disjoints = new LinkedList<String>();
		List<String> dd = new LinkedList<String>();

		for (dBelief bel : bm.values()) {
			if (bel.content instanceof BasicProbDistribution) {
				BasicProbDistribution bpd = (BasicProbDistribution)bel.content;

				dd.clear();
				if (bpd.values instanceof FormulaValues) {
					FormulaValues fv = (FormulaValues)bpd.values;
					for (FormulaProbPair fp : fv.values) {
						args.add("(bel : w('" + bel.id + "','" + bpd.key + "')) = p(" + fp.prob + ")");
						dd.add("bel : w('" + bel.id + "','" + bpd.key + "')");
					}
				}
				disjoints.add("disjoint([" + join(", ", dd) + "]).\n");
			}
		}
		sb.append("world_exist = [\n\t").append(join(",\n\t", args)).append("\n].\n");
		sb.append("\n");
		sb.append(join("", disjoints));
		sb.append("\n");

		// examine each world and properties it implies
		args.clear();
		for (dBelief bel : bm.values()) {
			if (bel.content instanceof BasicProbDistribution) {
				BasicProbDistribution bpd = (BasicProbDistribution)bel.content;

				if (bpd.values instanceof FormulaValues) {
					FormulaValues fv = (FormulaValues)bpd.values;
					for (FormulaProbPair fp : fv.values) {
						String tail = " <- bel : w('" + bel.id + "', '" + bpd.key + "') / world_exist, bel : b('" + bel.id + "') / belief_exist.";
						expandFormulaToArgsWithIdTail(fp.val, args, bel.id, tail);
					}
				}
			}
		}
		sb.append(join("\n", args));

		try {
			BufferedWriter f = new BufferedWriter(new FileWriter(dumpfile));
			f.write(sb.toString());
			f.close();
		}
		catch (IOException ex) {
			log("I/O error while writing the dumpfile");
		}
	}

	private void expandFormulaToArgsWithIdTail(dFormula val, List<String> args, String id, String tail) {
		if (val instanceof ComplexFormula && ((ComplexFormula)val).op == BinaryOp.conj) {
			ComplexFormula cF = (ComplexFormula)val;
			for (dFormula subF : cF.forms) {
				expandFormulaToArgsWithIdTail(subF, args, id, tail);
			}
		}
		else if (val instanceof ModalFormula) {
			ModalFormula mF = (ModalFormula)val;
			if (mF.form instanceof ElementaryFormula) {
				ElementaryFormula eF = (ElementaryFormula)mF.form;
				String s = MercuryUtils.termStringEscape(mF.op.toLowerCase()) + "('" + id + "','" + eF.prop + "')";
				args.add(s + tail);
			}
		}
	}

	// Sun should burn in hell for not having such a function in the standard library!
	public static String join(String separator, List<String> args) {
	    StringBuilder sb = new StringBuilder();
	    if (args != null) {
			Iterator<String> iter = args.iterator();
			while (iter.hasNext()) {
				sb.append(iter.next());
				if (iter.hasNext()) {
					sb.append(separator);
				}
			}
		}
		return sb.toString();
    }

	@Override
	public void executeTask(ProcessingData data) throws DialogueException {
		// don't care about the data really
		rewriteRulefile(dumpfile);
	}

}
