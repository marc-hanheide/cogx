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

import de.dfki.lt.tr.dialogue.interpret.atoms.AssertedReferenceAtom;
import de.dfki.lt.tr.dialogue.util.BeliefFormulaFactory;
import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
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
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequest;
import de.dfki.lt.tr.dialogue.ref.ReferenceResolutionRequestExtractor;
import de.dfki.lt.tr.dialogue.slice.lf.LogicalForm;
import de.dfki.lt.tr.dialogue.time.TimeInterval;
import de.dfki.lt.tr.dialogue.util.Counter;
import de.dfki.lt.tr.dialogue.util.IdentifierGenerator;
import de.dfki.lt.tr.infer.abducer.lang.FunctionTerm;
import de.dfki.lt.tr.infer.abducer.lang.ModalisedAtom;
import de.dfki.lt.tr.infer.abducer.lang.Modality;
import de.dfki.lt.tr.infer.abducer.lang.Term;
import de.dfki.lt.tr.infer.abducer.lang.VariableTerm;
import de.dfki.lt.tr.infer.abducer.proof.MarkedQuery;
import de.dfki.lt.tr.infer.abducer.util.PrettyPrint;
import de.dfki.lt.tr.infer.abducer.util.ProofUtils;
import de.dfki.lt.tr.infer.abducer.util.TermAtomFactory;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import org.apache.log4j.Logger;
//import org.cognitivesystems.binder.POINTERLABEL;

public abstract class ConversionUtils {

	private static Logger logger = Logger.getLogger("conversion-utils");

//	public static String commitSA = "memory";
//	public static String dialogueSA = "dialogue";

	private static Counter counter = new Counter("conv");

	public static final String predsym_UTTER = "utter";
	public static final String predsym_BELIEF = "belief";
	public static final String predsym_AGENT = "agent";
	public static final String predsym_PRECONDITION = "pre";
	public static final String predsym_POSTCONDITION = "post";
	public static final String predsym_RESOLVES_TO_BELIEF = "resolves_to_belief";
	public static final String predsym_RESOLVES_TO_EPOBJECT = "resolves_to_epobject";
	public static final String predsym_TOPIC_SWITCH_MARK = "topic_switch_mark";
	public static final String predsym_POINTER = "pointer";

	public static final String functor_EPST_PRIVATE = "private";
	public static final String functor_EPST_ATTRIB = "attrib";
	public static final String functor_EPST_SHARED = "shared";
	public static final String functor_FEATVAL = "fv";
	public static final String functor_PTR = "ptr";
	public static final String functor_VAR = "var";
	public static final String functor_OR = "or";
	public static final String functor_NOT = "not";
	public static final String functor_RPV = "rpv";
	public static final String functor_POINTER_TO = "pointer-to";
	public static final String functor_MARK = "mark";

	public static final String feat_MARK = "mark";

	private static final String NIL = "nil";

	public static final String POINTERLABEL = "about";

	public static dFormula replacePointersInFormula(dFormula f, List<dBelief> bels, String subarch) {
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
			if (mF.op.equals(functor_POINTER_TO) && mF.form instanceof ElementaryFormula) {
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

	public static dBelief findMarkedBelief(List<dBelief> bels, String mark) {
		for (dBelief b : bels) {
			if (b.content instanceof CondIndependentDistribs) {
				CondIndependentDistribs cnt = (CondIndependentDistribs) b.content;
				ProbDistribution pd = cnt.distribs.get(feat_MARK);
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

	public static void addFeature(dBelief b, String featName, dFormula f) {
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

	public static void removeFeature(dBelief b, String featName) {
		if (b.content instanceof CondIndependentDistribs) {
			CondIndependentDistribs cnt = (CondIndependentDistribs) b.content;
			cnt.distribs.remove(featName);
		}
	}

	public static List<Term> listTermToListOfTerms(Term t) {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals("[]") && ft.args.isEmpty()) {
				return new LinkedList<Term>();
			}
			if (ft.functor.equals("[|]") && ft.args.size() == 2) {
				List<Term> result = new LinkedList<Term>();
				result.add(ft.args.get(0));
				List<Term> tail = listTermToListOfTerms(ft.args.get(1));
				if (tail == null) {
					return null;
				}
				else {
					result.addAll(tail);
				}
				return result;
			}
		}
		return null;
	}

	public static Term listStringsToTerm(List<String> l) {
		List<Term> terms = new LinkedList<Term>();
		for (String s : l) {
			terms.add(TermAtomFactory.term(s));
		}
		return listToTerm(terms);
	}

	public static dFormula uniTermToFormula(Term t) {

		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;

			// empty list
			if (ft.functor.equals("[]") && ft.args.isEmpty()) {
				return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, new LinkedList<dFormula>());
			}

			// list head
			if (ft.functor.equals("[|]") && ft.args.size() == 2) {
				List<dFormula> lfs = new LinkedList<dFormula>();
				lfs.add(uniTermToFormula(ft.args.get(0)));
				dFormula tail = uniTermToFormula(ft.args.get(1));
				assert (tail instanceof ComplexFormula);
				ComplexFormula cF = (ComplexFormula) tail;
				lfs.addAll(cF.forms);
				return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, lfs);
			}

			// pointer to an address
			if (ft.functor.equals(functor_PTR)) {
				WorkingMemoryAddress wma = termToWorkingMemoryAddress(ft);
				if (wma != null) {
					return BeliefFormulaFactory.newPointerFormula(wma);
				}
			}

			if (ft.functor.equals(functor_VAR)) {
				return termToUnderspecifiedFormula(ft);
			}

			// modal formula
			if (ft.functor.equals(functor_NOT) && ft.args.size() == 1) {
				return BeliefFormulaFactory.newNegatedFormula(uniTermToFormula(ft.args.get(0)));
			}

			// modal formula
			if (ft.args.size() == 1) {
				return BeliefFormulaFactory.newModalFormula(ft.functor, uniTermToFormula(ft.args.get(0)));
			}

			// elementary formula
			if (ft.args.isEmpty()) {
				return BeliefFormulaFactory.newElementaryFormula(ft.functor);
			}

			return BeliefFormulaFactory.newElementaryFormula(NIL);
		}
		else if (t instanceof VariableTerm) {
			VariableTerm vt = (VariableTerm) t;
			return new UnderspecifiedFormula(-1, vt.name);
		}
		return null;
 	}

	public static EpistemicStatus termToEpistemicStatus(FunctionTerm epstT) {
		if (epstT.functor.equals(functor_EPST_PRIVATE)) {
			PrivateEpistemicStatus p = new PrivateEpistemicStatus();
			p.agent = ((FunctionTerm)epstT.args.get(0)).functor;
			return p;
		}
		if (epstT.functor.equals(functor_EPST_ATTRIB)) {
			AttributedEpistemicStatus a = new AttributedEpistemicStatus();
			a.agent = ((FunctionTerm)epstT.args.get(0)).functor;
			a.attribagents = new LinkedList<String>();
			a.attribagents.add(((FunctionTerm)epstT.args.get(1)).functor);
			return a;
		}
		if (epstT.functor.equals(functor_EPST_SHARED)) {
			SharedEpistemicStatus s = new SharedEpistemicStatus();
			s.cgagents = new LinkedList<String>();
			for (Term t : epstT.args) {
				String ag = ProofUtils.termFunctor(t);
				if (ag != null) {
					s.cgagents.add(ag);
				}
			}
			return s;
		}
		return null;
	}

	public static dBelief newEmptyCondIndepDistribBelief(String id, EpistemicStatus epst) {
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
			return TermAtomFactory.term(functor_EPST_PRIVATE, new Term[] {
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
			return TermAtomFactory.term(functor_EPST_ATTRIB, args.toArray(new Term[0]));
		}
		if (epst instanceof SharedEpistemicStatus) {
			SharedEpistemicStatus a = (SharedEpistemicStatus)epst;
			LinkedList<FunctionTerm> args = new LinkedList<FunctionTerm>();
			for (String ag : a.cgagents) {
				args.add(TermAtomFactory.term(ag));
			}
			return TermAtomFactory.term(functor_EPST_SHARED, args.toArray(new Term[0]));
		}
		return null;
	}

	public static IntentionalContent newIntentionalContent(float probValue) {
		IntentionalContent itc = new IntentionalContent();
		itc.probValue = probValue;
		itc.preconditions = BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, new LinkedList<dFormula>());
		itc.postconditions = BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, new LinkedList<dFormula>());
		return itc;
	}

	public static String foldIntoBeliefs(IdentifierGenerator<String> idGen, EpistemicStatus epst, String lingRef, Map<String, WorkingMemoryAddress> usedRefs, FunctionTerm t, List<dBelief> bel) {
		Iterator<dBelief> it = bel.iterator();
		while (it.hasNext()) {
			dBelief xb = it.next();
			if (foldTermAsContent(epst, lingRef, t, xb)) {
				return null;
			}
		}
		// we need to add a new belief
		String newId = idGen.newIdentifier();
		dBelief b = newEmptyCondIndepDistribBelief(newId, epst);
		// XXX we should actually check success here
		foldTermAsContent(epst, lingRef, TermAtomFactory.term(functor_FEATVAL, new Term[] {TermAtomFactory.term(IntentionManagementConstants.discRefModality), TermAtomFactory.term(lingRef)}), b);
		if (usedRefs.containsKey(lingRef)) {
			foldTermAsContent(epst, lingRef, TermAtomFactory.term(functor_FEATVAL, new Term[] {TermAtomFactory.term(POINTERLABEL), workingMemoryAddressToTerm(usedRefs.get(lingRef))}), b);
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
		if (t.functor.equals(functor_FEATVAL)) {
			featureName = ((FunctionTerm)t.args.get(0)).functor;
			featureValue = functionTermToContentFormula((FunctionTerm)t.args.get(1));

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
		if (t.functor.equals(functor_MARK)) {
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

	public static dFormula functionTermToContentFormula(FunctionTerm t) {
		if (t.functor.equals(functor_PTR)) {
			WorkingMemoryAddress wma = termToWorkingMemoryAddress(t);
			if (wma != null) {
				return BeliefFormulaFactory.newPointerFormula(wma);
			}
		}
		if (t.functor.equals(functor_NOT) && t.args.size() == 1) {
			return BeliefFormulaFactory.newNegatedFormula(functionTermToContentFormula((FunctionTerm)t.args.get(0)));
		}
		if (t.functor.equals(functor_OR) && t.args.size() == 2) {
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.disj, functionTermToContentFormula((FunctionTerm)t.args.get(0)), functionTermToContentFormula((FunctionTerm)t.args.get(1)));
		}
		else {
			return BeliefFormulaFactory.newElementaryFormula(t.functor);
		}
	}

	public static WorkingMemoryAddress termToWorkingMemoryAddress(Term t) {
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals(functor_PTR) && ft.args.size() == 2 && ft.args.get(0) instanceof FunctionTerm && ft.args.get(1) instanceof FunctionTerm) {
				String subarch = ((FunctionTerm) ft.args.get(0)).functor;
				String id = ((FunctionTerm) ft.args.get(1)).functor;
				return new WorkingMemoryAddress(id, subarch);
			}
		}
		return null;
	}

	public static UnderspecifiedFormula termToUnderspecifiedFormula(Term t) {
		String varName = null;
		if (t instanceof FunctionTerm) {
			FunctionTerm ft = (FunctionTerm) t;
			if (ft.functor.equals(functor_VAR) && ft.args.size() == 1) {
				Term argt = ft.args.get(0);
				if (argt instanceof FunctionTerm && ((FunctionTerm)argt).args.isEmpty()) {
					varName = ((FunctionTerm)argt).functor;
				}
			}
		}
		else if (t instanceof VariableTerm) {
			VariableTerm vt = (VariableTerm) t;
			varName = vt.name;
		}
		
		return (varName != null ? new UnderspecifiedFormula(-1, varName) : null);
	}

	public static Term workingMemoryAddressToTerm(WorkingMemoryAddress wma) {
		return TermAtomFactory.term(functor_PTR, new Term[] { TermAtomFactory.term(wma.subarchitecture), TermAtomFactory.term(wma.id)});
	}

	public static dFormula combineDFormulas(dFormula oldF, dFormula newF) {
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
					TermAtomFactory.atom(predsym_AGENT, args));

			log("adding fact: " + PrettyPrint.modalisedAtomToString(amf));
			result.add(amf);

			for (ModalisedAtom ma : intentionFormulaToFacts(itIdTerm, itc.preconditions, ms, predsym_PRECONDITION)) {
				log("adding fact: " + PrettyPrint.modalisedAtomToString(ma));
				result.add(ma);
			}

			for (ModalisedAtom ma : intentionFormulaToFacts(itIdTerm, itc.postconditions, ms, predsym_POSTCONDITION)) {
				log("adding fact: " + PrettyPrint.modalisedAtomToString(ma));
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
				else {
					// not a complex formula -> make a singleton list
					List<dFormula> sl = new LinkedList<dFormula>();
					sl.add(mf.form);
					FunctionTerm ft = TermAtomFactory.term(IntentionManagementConstants.stateModality, new Term[] {listToTerm(stateFormulaToTerms(sl))});
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

	public static Term stateFormulaToTerm(dFormula f) {
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

	static List<ReferenceResolutionRequest> extractReferenceRequests(ReferenceResolutionRequestExtractor extractor, LogicalForm lf, List<MarkedQuery> proof, TimeInterval ival) {
		List<ReferenceResolutionRequest> result = new ArrayList<ReferenceResolutionRequest>();

		List<ModalisedAtom> matoms = ProofUtils.filterStripByModalityPrefix(ProofUtils.stripMarking(proof), Arrays.asList(new Modality[] {Modality.Understanding}));
		for (ModalisedAtom matom : matoms) {
			AssertedReferenceAtom aratom = AssertedReferenceAtom.fromModalisedAtom(matom);
			if (aratom != null && aratom.getReferentTerm() instanceof VariableTerm) {
				result.add(extractor.extractReferenceResolutionRequest(lf, aratom.getNominal(), ival));
			}
		}
		return result;
	}

	private static void log(String str) {
		if (logger != null) {
			logger.debug(str);
		}
	}
}
