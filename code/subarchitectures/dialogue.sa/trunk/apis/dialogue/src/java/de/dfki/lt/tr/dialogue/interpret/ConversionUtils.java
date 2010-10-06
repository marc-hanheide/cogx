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

package de.dfki.lt.tr.dialogue.interpret;

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.epobject.EpistemicObject;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;
import de.dfki.lt.tr.beliefs.slice.history.AbstractBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.util.Counter;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.infer.weigabd.MercuryUtils;
import de.dfki.lt.tr.infer.weigabd.ProofUtils;
import de.dfki.lt.tr.infer.weigabd.TermAtomFactory;
import de.dfki.lt.tr.infer.weigabd.slice.FunctionTerm;
import de.dfki.lt.tr.infer.weigabd.slice.MarkedQuery;
import de.dfki.lt.tr.infer.weigabd.slice.ModalisedAtom;
import de.dfki.lt.tr.infer.weigabd.slice.Modality;
import de.dfki.lt.tr.infer.weigabd.slice.Term;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.cognitivesystems.binder.POINTERLABEL;

public abstract class ConversionUtils {

	public static boolean logging = true;

	private static Counter counter = new Counter("conv");

	public static final String beliefPredSym = "belief";
	public static final String agentPredSym = "agent";
	public static final String preconditionPredSym = "pre";
	public static final String postconditionPredSym = "post";

	private static final String privateEpStFunctor = "private";
	private static final String attributedEpStFunctor = "attrib";
	private static final String sharedEpStFunctor = "shared";

	public static RecognisedIntention proofToEpistemicObjects(IdentifierGenerator idGen, String attribAgent, MarkedQuery[] proof) {

		LinkedList<EpistemicObject> results = new LinkedList<EpistemicObject>();
		List<dBelief> bels_pre = new LinkedList<dBelief>();
		List<dBelief> bels_post = new LinkedList<dBelief>();

		Map<String, dBelief> marks = new HashMap<String, dBelief>();
		List<InterBeliefPointer> pointers = new LinkedList<InterBeliefPointer>();

		RecognisedIntention ri = new RecognisedIntention();

		Map<String, WorkingMemoryAddress> usedRefs = new HashMap<String, WorkingMemoryAddress>();
		ModalisedAtom[] rrs = ProofUtils.stripMarking(ProofUtils.filterAssumed(proof));

		for (ModalisedAtom ma : Arrays.asList(rrs)) {
			if (ma.m.length == 1 && ma.m[0] == Modality.Understanding && ma.a.predSym.equals("resolves_to_belief") && ma.a.args.length == 2) {
				String n = ((FunctionTerm)ma.a.args[0]).functor;
				WorkingMemoryAddress wma = termToWorkingMemoryAddress(ma.a.args[1]);
				if (wma != null) {
					usedRefs.put(n, wma);
				}
			}
		}

		ModalisedAtom[] imfs = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(ProofUtils.filterAssumed(proof)),
				new Modality[] {Modality.Intention});

		HashMap<String, IntentionalContent> rIts = new HashMap<String, IntentionalContent>();

		// TODO: make a dedicated class for this
		for (ModalisedAtom ma : Arrays.asList(imfs)) {
			if (ma.a.predSym.equals(agentPredSym)) {
				FunctionTerm idTerm = (FunctionTerm) ma.a.args[0];
				FunctionTerm agentTerm = (FunctionTerm) ma.a.args[1];

				if (!rIts.containsKey(idTerm.functor)) {
					rIts.put(idTerm.functor, newIntentionalContent());
				}
				IntentionalContent itc = rIts.get(idTerm.functor);

				itc.agents = new LinkedList<String>();
				itc.agents.add(agentTerm.functor);
			}
			else if (ma.a.predSym.equals(preconditionPredSym)) {
				FunctionTerm idTerm = (FunctionTerm) ma.a.args[0];
				FunctionTerm argTerm = (FunctionTerm) ma.a.args[1];

				if (!rIts.containsKey(idTerm.functor)) {
					rIts.put(idTerm.functor, newIntentionalContent());
				}
				IntentionalContent itc = rIts.get(idTerm.functor);

				if (argTerm.functor.equals(IntentionManagementConstants.beliefLinkModality)) {
					String lingRef = ((FunctionTerm)argTerm.args[0]).functor;
					EpistemicStatus es = termToEpistemicStatus((FunctionTerm)argTerm.args[1]);

					FunctionTerm action = (FunctionTerm)argTerm.args[2];

					if (action.functor.equals("fv")) {
						String newId = foldIntoBeliefs(idGen, es, lingRef, usedRefs, action, bels_pre);
						if (newId != null) {
							dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.beliefLinkModality, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(newId, "binder")));
							itc.preconditions = combineDFormulas(itc.preconditions, refF);
						}
					}
/*
					else if (action.functor.equals("mark")) {
						String marking = ((FunctionTerm)action.args[0]).functor;
						String newId = foldIntoBeliefs(idGen, es, lingRef, usedRefs, action, bels_pre);
						if (newId != null) {
							dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagement.beliefLinkModality, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(newId, "binder")));
							itc.preconditions = combineDFormulas(itc.preconditions, refF);
						}
					}
 */
				}
				if (argTerm.functor.equals(IntentionManagementConstants.stateModality) && argTerm.args.length == 1) {
					dFormula stateF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.stateModality, uniTermToFormula((FunctionTerm) argTerm.args[0]));
					itc.preconditions = combineDFormulas(itc.preconditions, stateF);
				}
			}
			else if (ma.a.predSym.equals(postconditionPredSym)) {
				FunctionTerm idTerm = (FunctionTerm) ma.a.args[0];
				FunctionTerm argTerm = (FunctionTerm) ma.a.args[1];

				if (!rIts.containsKey(idTerm.functor)) {
					rIts.put(idTerm.functor, newIntentionalContent());
				}
				IntentionalContent itc = rIts.get(idTerm.functor);

				if (argTerm.functor.equals(IntentionManagementConstants.beliefLinkModality)) {
					String lingRef = ((FunctionTerm)argTerm.args[0]).functor;
					EpistemicStatus es = termToEpistemicStatus((FunctionTerm)argTerm.args[1]);

					FunctionTerm action = (FunctionTerm)argTerm.args[2];

					if (action.functor.equals("fv")) {
						String newId = foldIntoBeliefs(idGen, es, lingRef, usedRefs, action, bels_post);
						if (newId != null) {
							dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.beliefLinkModality, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(newId, "dialogue")));
							itc.postconditions = combineDFormulas(itc.postconditions, refF);
						}
					}
				}
				if (argTerm.functor.equals(IntentionManagementConstants.stateModality) && argTerm.args.length == 1) {
					dFormula stateF = BeliefFormulaFactory.newModalFormula(IntentionManagementConstants.stateModality, uniTermToFormula((FunctionTerm) argTerm.args[0]));
					itc.postconditions = combineDFormulas(itc.postconditions, stateF);
				}
			}
		}

		for (ModalisedAtom ma : Arrays.asList(rrs)) {
			if (ma.m.length == 1 && ma.m[0] == Modality.Intention && ma.a.predSym.equals("pointer") && ma.a.args.length == 3) {
				String featName = ((FunctionTerm)ma.a.args[0]).functor;
				String markFrom = ((FunctionTerm)ma.a.args[1]).functor;
				String markTo = ((FunctionTerm)ma.a.args[2]).functor;

				dBelief from = findMarkedBelief(bels_pre, markFrom);
//				if (from == null) {
//					from = findMarkedBelief(bels_post, markFrom);
//				}

				dBelief to = findMarkedBelief(bels_pre, markTo);
//				if (to == null) {
//					to = findMarkedBelief(bels_post, markTo);
//				}

				if (from != null && to != null) {
					addFeature(from, featName, BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(to.id, "binder")));  // FIXME: this is *very* hacky!
				}
			}
		}

		Iterator<String> iter = rIts.keySet().iterator();
		while (iter.hasNext()) {
			Intention it = new Intention();
			it.id = idGen.newIdentifier();
			it.frame = new AbstractFrame();
			AttributedEpistemicStatus epst = new AttributedEpistemicStatus();
			epst.agent = IntentionManagementConstants.thisAgent;
			epst.attribagents = new LinkedList<String>();
			epst.attribagents.add(attribAgent);
			it.estatus = epst;
			it.content = new LinkedList<IntentionalContent>();
			IntentionalContent itc = rIts.get(iter.next());
			it.content.add(itc);
			if (itc.agents != null && !itc.agents.isEmpty()) {

				// replace pointers in the formulas
				replacePointersInFormula(itc.preconditions, bels_pre, "binder");
				replacePointersInFormula(itc.preconditions, bels_post, "dialogue");
				replacePointersInFormula(itc.postconditions, bels_pre, "binder");
				replacePointersInFormula(itc.postconditions, bels_post, "dialogue");

				ri.ints.add(it);
			}
			else {
				log("discarding [" + it.id + "]: incomplete (agent list empty)");
			}
		}

		for (dBelief b : bels_pre) {
			removeFeature(b, "mark");
		}
		for (dBelief b : bels_post) {
			removeFeature(b, "mark");
		}

		ri.pre.addAll(bels_pre);
		ri.post.addAll(bels_post);

		return ri;
	}

	private static dFormula replacePointersInFormula(dFormula f, List<dBelief> bels, String subarch) {
		if (f instanceof ComplexFormula) {
			ComplexFormula cF = (ComplexFormula) f;
			for (dFormula ff : cF.forms) {
				dFormula repl = replacePointersInFormula(ff, bels, subarch);
				if (repl != null) {
					ff = repl;
				}
			}
		}
		if (f instanceof ModalFormula) {
			ModalFormula mF = (ModalFormula) f;
			if (mF.op.equals("pointer-to") && mF.form instanceof ElementaryFormula) {
				ElementaryFormula eF = (ElementaryFormula) mF.form;
				dBelief to = findMarkedBelief(bels, eF.prop);
				if (to != null) {
					return BeliefFormulaFactory.newPointerFormula(new WorkingMemoryAddress(to.id, subarch));
				}
			}
			else {
				dFormula repl =	replacePointersInFormula(mF.form, bels, subarch);
				if (repl != null) {
					mF.form = repl;
				}
			}
		}
		return null;
	}

	private static dBelief findMarkedBelief(List<dBelief> bels, String mark) {
		for (dBelief b : bels) {
			if (b.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs cnt = (CondIndependentDistribs) b.content;
				ProbDistribution pd = cnt.distribs.get("mark");
				if (pd != null && pd instanceof BasicProbDistribution) {
					BasicProbDistribution bpd = (BasicProbDistribution) pd;
					if (bpd.values instanceof FormulaValues) {
						FormulaValues fvs = (FormulaValues) bpd.values;
						for (FormulaProbPair fpp : fvs.values) {
							if (fpp.val instanceof ElementaryFormula && ((ElementaryFormula)fpp.val).prop.equals(mark)) {
								return b;
							}
						}
					}
				}
			}
		}
		return null;
	}

	private static void addFeature(dBelief b, String featName, dFormula f) {
		if (b.content instanceof CondIndependentDistribs) {
			CondIndependentDistribs cnt = (CondIndependentDistribs) b.content;

			FormulaProbPair fpp = new FormulaProbPair();
			fpp.prob = 1.0f;
			fpp.val = f;

			FormulaValues fvs = new FormulaValues();
			fvs.values = new LinkedList<FormulaProbPair>();
			fvs.values.add(fpp);

			BasicProbDistribution bpd = new BasicProbDistribution();
			bpd.key = featName;
			bpd.values = fvs;

			cnt.distribs.put(featName, bpd);
		}
	}

	private static void removeFeature(dBelief b, String featName) {
		if (b.content instanceof CondIndependentDistribs) {
			CondIndependentDistribs cnt = (CondIndependentDistribs) b.content;
			cnt.distribs.remove(featName);
		}
	}

	private static dFormula uniTermToFormula(FunctionTerm ft) {

		// empty list
		if (ft.functor.equals("[]") && ft.args.length == 0) {
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, new LinkedList<dFormula>());
		}

		// list head
		if (ft.functor.equals("[|]") && ft.args.length == 2) {
			List<dFormula> lfs = new LinkedList<dFormula>();
			lfs.add(uniTermToFormula((FunctionTerm) ft.args[0]));
			dFormula tail = uniTermToFormula((FunctionTerm) ft.args[1]);
			assert (tail instanceof ComplexFormula);
			ComplexFormula cF = (ComplexFormula) tail;
			lfs.addAll(cF.forms);
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, lfs);
		}

		// pointer to an address
		if (ft.functor.equals("ptr")) {
			WorkingMemoryAddress wma = termToWorkingMemoryAddress(ft);
			if (wma != null) {
				return BeliefFormulaFactory.newPointerFormula(wma);
			}
		}

		// modal formula
		if (ft.args.length == 1) {
			return BeliefFormulaFactory.newModalFormula(ft.functor, uniTermToFormula((FunctionTerm) ft.args[0]));
		}

		// elementary formula
		if (ft.args.length == 0) {
			return BeliefFormulaFactory.newElementaryFormula(ft.functor);
		}

		return BeliefFormulaFactory.newElementaryFormula("nil");
 	}

	public static EpistemicStatus termToEpistemicStatus(FunctionTerm epstT) {
		if (epstT.functor.equals(privateEpStFunctor)) {
			PrivateEpistemicStatus p = new PrivateEpistemicStatus();
			p.agent = ((FunctionTerm)epstT.args[0]).functor;
			return p;
		}
		if (epstT.functor.equals(attributedEpStFunctor)) {
			AttributedEpistemicStatus a = new AttributedEpistemicStatus();
			a.agent = ((FunctionTerm)epstT.args[0]).functor;
			a.attribagents = new LinkedList<String>();
			a.attribagents.add(((FunctionTerm)epstT.args[1]).functor);
			return a;
		}
		if (epstT.functor.equals(sharedEpStFunctor)) {
			SharedEpistemicStatus s = new SharedEpistemicStatus();
			s.cgagents = new LinkedList<String>();
			for (int i = 0; i < epstT.args.length; i++) {
				s.cgagents.add(((FunctionTerm)epstT.args[i]).functor);
			}
			return s;
		}
		return null;
	}

	private static dBelief emptyCondIndepDistribBelief(String id, EpistemicStatus epst) {
		dBelief b = new dBelief();
		b.frame = new AbstractFrame();
		b.id = id;
		b.estatus = epst;
		b.type = "fact";
		b.hist = new AbstractBeliefHistory();
		CondIndependentDistribs ds = new CondIndependentDistribs();
		ds.distribs = new HashMap<String, ProbDistribution>();
		b.content = ds;
		return b;
	}

	public static FunctionTerm epistemicStatusToTerm(EpistemicStatus epst) {
		if (epst instanceof PrivateEpistemicStatus) {
			PrivateEpistemicStatus p = (PrivateEpistemicStatus)epst;
			return TermAtomFactory.term(privateEpStFunctor, new Term[] {
					TermAtomFactory.term(p.agent)
				});
		}
		if (epst instanceof AttributedEpistemicStatus) {
			AttributedEpistemicStatus a = (AttributedEpistemicStatus)epst;
			LinkedList<FunctionTerm> args = new LinkedList<FunctionTerm>();
			args.add(TermAtomFactory.term(a.agent));
			for (String ag : a.attribagents) {
				args.add(TermAtomFactory.term(ag));
			}
			return TermAtomFactory.term(attributedEpStFunctor, args.toArray(new Term[0]));
		}
		if (epst instanceof SharedEpistemicStatus) {
			SharedEpistemicStatus a = (SharedEpistemicStatus)epst;
			LinkedList<FunctionTerm> args = new LinkedList<FunctionTerm>();
			for (String ag : a.cgagents) {
				args.add(TermAtomFactory.term(ag));
			}
			return TermAtomFactory.term(sharedEpStFunctor, args.toArray(new Term[0]));
		}
		return null;
	}

	private static IntentionalContent newIntentionalContent() {
		IntentionalContent itc = new IntentionalContent();
		itc.probValue = 1.0f;
		itc.preconditions = BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, new LinkedList<dFormula>());
		itc.postconditions = BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, new LinkedList<dFormula>());
		return itc;
	}

	private static String foldIntoBeliefs(IdentifierGenerator idGen, EpistemicStatus epst, String lingRef, Map<String, WorkingMemoryAddress> usedRefs, FunctionTerm t, List<dBelief> bel) {
		Iterator<dBelief> it = bel.iterator();
		while (it.hasNext()) {
			dBelief xb = it.next();
			if (foldTermAsContent(epst, lingRef, t, xb)) {
				return null;
			}
		}
		// we need to add a new belief
		String newId = idGen.newIdentifier();
		dBelief b = emptyCondIndepDistribBelief(newId, epst);
		// XXX we should actually check success here
		foldTermAsContent(epst, lingRef, TermAtomFactory.term("fv", new Term[] {TermAtomFactory.term(IntentionManagementConstants.discRefModality), TermAtomFactory.term(lingRef)}), b);
		if (usedRefs.containsKey(lingRef)) {
			foldTermAsContent(epst, lingRef, TermAtomFactory.term("fv", new Term[] {TermAtomFactory.term(POINTERLABEL.value), workingMemoryAddressToTerm(usedRefs.get(lingRef))}), b);
		}
		foldTermAsContent(epst, lingRef, t, b);
		bel.add(b);
		return newId;
	}

	/**
	 * Fold the interpretation of the term as a feature-value specification
	 * into a belief. Return false when this isn't possible, true otherwise.
	 *
	 * @param epst epistemic status
	 * @param lingRef linguistic referent
	 * @param t the term
	 * @param bel the belief
	 * @return false if this didn't happen, true otherwise
	 */
	private static boolean foldTermAsContent(EpistemicStatus epst, String lingRef, FunctionTerm t, dBelief bel) {
		String featureName = "";
		dFormula featureValue;
		if (t.functor.equals("fv")) {
			featureName = ((FunctionTerm)t.args[0]).functor;
			featureValue = termToContentFormula((FunctionTerm)t.args[1]);

			Comparator<EpistemicStatus> cmp = new EpistemicStatusComparator();
			if (cmp.compare(bel.estatus, epst) != 0) {
				return false;
			}

			if (bel.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs ds = (CondIndependentDistribs)bel.content;
				ds.distribs.put(featureName, logicalBasicProbDistribution(featureName, featureValue));
				return true;
			}
		}
		if (t.functor.equals("mark")) {
			return true;
		}
 		return false;
	}

	private static BasicProbDistribution logicalBasicProbDistribution(String featName, dFormula featValue) {
		BasicProbDistribution bpd = new BasicProbDistribution();
		bpd.key = featName;

		FormulaProbPair log = new FormulaProbPair();
		log.prob = 1.0f;
		log.val = featValue;

		FormulaValues fv = new FormulaValues();
		fv.values = new LinkedList<FormulaProbPair>();
		fv.values.add(log);

		bpd.values = fv;

		return bpd;
	}

	private static dFormula termToContentFormula(FunctionTerm t) {
		if (t.functor.equals("ptr")) {
			WorkingMemoryAddress wma = termToWorkingMemoryAddress(t);
			if (wma != null) {
				return BeliefFormulaFactory.newPointerFormula(wma);
			}
		}
		if (t.functor.equals("not") && t.args.length == 1) {
			return BeliefFormulaFactory.newNegatedFormula(termToContentFormula((FunctionTerm)t.args[0]));
		}
		if (t.functor.equals("or") && t.args.length == 2) {
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.disj, termToContentFormula((FunctionTerm)t.args[0]), termToContentFormula((FunctionTerm)t.args[1]));
		}
		else {
			return BeliefFormulaFactory.newElementaryFormula(t.functor);
		}
	}

	private static WorkingMemoryAddress termToWorkingMemoryAddress(Term t) {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals("ptr") && ft.args.length == 2 && ft.args[0] instanceof FunctionTerm && ft.args[1] instanceof FunctionTerm) {
				String subarch = ((FunctionTerm) ft.args[0]).functor;
				String id = ((FunctionTerm) ft.args[1]).functor;
				return new WorkingMemoryAddress(id, subarch);
			}
		}
		return null;
	}

	public static Term workingMemoryAddressToTerm(WorkingMemoryAddress wma) {
		return TermAtomFactory.term("ptr", new Term[] { TermAtomFactory.term(wma.subarchitecture), TermAtomFactory.term(wma.id)});
	}

	private static dFormula combineDFormulas(dFormula oldF, dFormula newF) {
		if (oldF == null) {
			return newF;
		}
		if (newF == null) {
			return oldF;
		}

		// try to have oldF a conjunction
		if (newF instanceof ComplexFormula && ((ComplexFormula)newF).op == BinaryOp.conj) {
			dFormula tmp = newF;
			newF = oldF;
			oldF = tmp;
		}

		if (oldF instanceof ComplexFormula && ((ComplexFormula)oldF).op == BinaryOp.conj) {
			ComplexFormula c_oldF = (ComplexFormula)oldF;
			List<dFormula> newFs = new LinkedList<dFormula>();
			newFs.addAll(c_oldF.forms);

			if (newF instanceof ComplexFormula && ((ComplexFormula)newF).op == BinaryOp.conj) {
				ComplexFormula c_newF = (ComplexFormula)newF;
				newFs.addAll(c_newF.forms);
			}
			else {
				newFs.add(newF);
			}
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, newFs);
		}
		else {
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, oldF, newF);
		}
	}

	private static dFormula getFirstLogicalContent(dBelief b) {
		if (b.content instanceof BasicProbDistribution) {
			BasicProbDistribution pd = (BasicProbDistribution)b.content;
			if (pd.values instanceof FormulaValues) {
				FormulaValues fv = (FormulaValues)pd.values;
				Iterator<FormulaProbPair> i = fv.values.iterator();
				while (i.hasNext()) {
					FormulaProbPair fp = i.next();
					return fp.val;
				}
			}
		}
		return null;
	}

	/* FIXME: this is very ugly and should be rewritten */
	private static FunctionTerm dFormulaToRPV(dFormula f) {
		String objRef = "";
		String objProp = "";
		FunctionTerm valTerm = null;
		String propVal = "";
		if (f instanceof ComplexFormula) {
			ComplexFormula cplxF = (ComplexFormula)f;
			if (cplxF.forms.size() == 2) {
				dFormula arg1 = cplxF.forms.get(0);
				dFormula arg2 = cplxF.forms.get(1);
				if (arg1 instanceof ModalFormula && arg2 instanceof ModalFormula) {
					ModalFormula m1 = (ModalFormula)arg1;
					ModalFormula m2 = (ModalFormula)arg2;
					if (m1.op.equals(IntentionManagementConstants.discRefModality) && m1.form instanceof ElementaryFormula) {
						ElementaryFormula eF = (ElementaryFormula)m1.form;
						objRef = eF.prop;
					}
					else {
						return null;
					}
					objProp = m2.op;
					if (m2.form instanceof ElementaryFormula) {
						ElementaryFormula eF = (ElementaryFormula)m2.form;
						valTerm = TermAtomFactory.term(eF.prop);
					}
					else if (m2.form instanceof NegatedFormula) {
						NegatedFormula nF = (NegatedFormula)m2.form;
						if (nF.negForm instanceof ElementaryFormula) {
							ElementaryFormula eF = (ElementaryFormula)nF.negForm;
							valTerm = TermAtomFactory.term("not", new Term[] {TermAtomFactory.term(eF.prop)});
						}
						else {
							return null;
						}
					}
					else {
						return null;
					}
					return TermAtomFactory.term("rpv", new Term[] {
						TermAtomFactory.term(objRef),
						TermAtomFactory.term(objProp),
						valTerm
					});
				}
			}
		}
		return null;
	}


	public static Term listToTerm(List<Term> ts) {
		return iteratorToTerm(ts.iterator());
	}

	public static FunctionTerm iteratorToTerm(Iterator<Term> i) {
		if (i.hasNext()) {
			Term t = i.next();
			FunctionTerm ft = TermAtomFactory.term("[|]", new Term[] { t, iteratorToTerm(i) });
			return ft;
		}
		else {
			return TermAtomFactory.term("[]");
		}
	}

	public static List<ModalisedAtom> intentionToFacts(Term itIdTerm, Intention it) {
		final Modality[] ms = new Modality[] {
			Modality.Truth,
			Modality.Intention
		};

		List<ModalisedAtom> result = new LinkedList<ModalisedAtom>();

		for (IntentionalContent itc : it.content) {

			List<Term> tmp = new LinkedList<Term>();
			for (String ag : itc.agents) {
				tmp.add(TermAtomFactory.term(ag));
			}

			List<Term> args = new LinkedList<Term>();
			args.add(itIdTerm);
			args.add(listToTerm(tmp));

			ModalisedAtom amf = TermAtomFactory.modalisedAtom(
					new Modality[] {
						Modality.Truth,
						Modality.Intention
					},
					TermAtomFactory.atom(agentPredSym, args));

			log("adding fact: " + MercuryUtils.modalisedAtomToString(amf));
			result.add(amf);

			for (ModalisedAtom ma : intentionFormulaToFacts(itIdTerm, itc.preconditions, ms, preconditionPredSym)) {
				log("adding fact: " + MercuryUtils.modalisedAtomToString(ma));
				result.add(ma);
			}

			for (ModalisedAtom ma : intentionFormulaToFacts(itIdTerm, itc.postconditions, ms, postconditionPredSym)) {
				log("adding fact: " + MercuryUtils.modalisedAtomToString(ma));
				result.add(ma);
			}
		}
		return result;
	}

	private static List<ModalisedAtom> intentionFormulaToFacts(Term itIdTerm, dFormula f, Modality[] modality, String atomPrefix) {
		List<ModalisedAtom> result = new LinkedList<ModalisedAtom>();

		if (f instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) f;
			assert (cf.op == BinaryOp.conj);
			for (dFormula ff : cf.forms) {
				result.addAll(intentionFormulaToFacts(itIdTerm, ff, modality, atomPrefix));
			}
		}

		if (f instanceof ModalFormula) {
			ModalFormula mf = (ModalFormula) f;
			if (mf.op.equals(IntentionManagementConstants.beliefLinkModality)) {
				if (mf.form instanceof PointerFormula) {
					// link to a belief
					PointerFormula pf = (PointerFormula) mf.form;
					FunctionTerm ft = TermAtomFactory.term(IntentionManagementConstants.beliefLinkModality, new Term[] {workingMemoryAddressToTerm(pf.pointer)});
					result.add(TermAtomFactory.modalisedAtom(modality, TermAtomFactory.atom(atomPrefix, new Term[] {itIdTerm, ft})));
				}
			}

			if (mf.op.equals(IntentionManagementConstants.stateModality)) {
				if (mf.form instanceof ComplexFormula) {
					// a state
					ComplexFormula cf = (ComplexFormula) mf.form;
					assert (cf.op == BinaryOp.conj);
					FunctionTerm ft = TermAtomFactory.term(IntentionManagementConstants.stateModality, new Term[] {listToTerm(stateFormulaToTerms(cf.forms))});
					result.add(TermAtomFactory.modalisedAtom(modality, TermAtomFactory.atom(atomPrefix, new Term[] {itIdTerm, ft})));
				}
			}
		}

		return result;
	}

	private static List<Term> stateFormulaToTerms(List<dFormula> fs) {
		List<Term> result = new LinkedList<Term>();
		for (dFormula f : fs) {
			Term t = stateFormulaToTerm(f);
			if (t != null) {
				result.add(t);
			}
		}
		return result;
	}

	private static Term stateFormulaToTerm(dFormula f) {
		if (f instanceof ComplexFormula) {
			ComplexFormula cf = (ComplexFormula) f;
			assert (cf.op == BinaryOp.conj);
			List<Term> ts = new LinkedList<Term>();
			for (dFormula ff : cf.forms) {
				Term t = stateFormulaToTerm(ff);
				if (t != null) {
					ts.add(t);
				}
			}
			return listToTerm(ts);
		}
		if (f instanceof ModalFormula) {
			ModalFormula mf = (ModalFormula) f;
			Term t = stateFormulaToTerm(mf.form);
			if (t != null) {
				return TermAtomFactory.term(mf.op, new Term[] { t });
			}
		}
		if (f instanceof ElementaryFormula) {
			ElementaryFormula ef = (ElementaryFormula) f;
			return TermAtomFactory.term(ef.prop);
		}
		if (f instanceof PointerFormula) {
			PointerFormula pf = (PointerFormula) f;
			return workingMemoryAddressToTerm(pf.pointer);
		}

		return null;
	}

	public static List<ModalisedAtom> beliefToFacts(dBelief b) {
		LinkedList<ModalisedAtom> result = new LinkedList<ModalisedAtom>();
		FunctionTerm rvp = dFormulaToRPV(getFirstLogicalContent(b));
		if (rvp != null) {
			ModalisedAtom mf = TermAtomFactory.modalisedAtom(
					new Modality[] {
						Modality.Belief
					},
					TermAtomFactory.atom(beliefPredSym, new Term[] {
						TermAtomFactory.term(b.id),
						epistemicStatusToTerm(b.estatus),
						rvp
					}));
			log("adding fact: " + MercuryUtils.modalisedAtomToString(mf));
			result.add(mf);
		}
		else {
			log("failed to generate RVP");
		}
		return result;
	}

	private static void log(String str) {
		if (logging)
			System.out.println("\033[32m[Conversion]\t" + str + "\033[0m");
	}

}