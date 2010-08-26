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
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

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

	public static List<ModalisedAtom> intentionToFacts(Intention itn) {
		LinkedList<ModalisedAtom> result = new LinkedList<ModalisedAtom>();
		for (IntentionalContent itc : itn.content) {

			LinkedList<FunctionTerm> args = new LinkedList<FunctionTerm>();
			args.add(TermAtomFactory.term(itn.id));
			for (String ag : itc.agents) {
				args.add(TermAtomFactory.term(ag));
			}

			ModalisedAtom amf = TermAtomFactory.modalisedAtom(
					new Modality[] {
						Modality.Truth,
						Modality.Intention
					},
					TermAtomFactory.atom(agentPredSym, args.toArray(new Term[0])));

			log("adding fact: " + MercuryUtils.modalisedAtomToString(amf));
			result.add(amf);

			for (String id : IntentionUtils.collectBeliefIdsInDFormula(itc.preconditions)) {
				ModalisedAtom mf = TermAtomFactory.modalisedAtom(
						new Modality[] {
							Modality.Truth,
							Modality.Intention
						},
						TermAtomFactory.atom(preconditionPredSym, new Term[] {
							TermAtomFactory.term(itn.id),
							TermAtomFactory.term(id)
						}));
				log("adding fact: " + MercuryUtils.modalisedAtomToString(mf));
				result.add(mf);
			}

			for (String id : IntentionUtils.collectBeliefIdsInDFormula(itc.postconditions)) {
				ModalisedAtom mf = TermAtomFactory.modalisedAtom(
						new Modality[] {
							Modality.Truth,
							Modality.Intention
						},
						TermAtomFactory.atom(postconditionPredSym, new Term[] {
							TermAtomFactory.term(itn.id),
							TermAtomFactory.term(id)
						}));
				log("adding fact: " + MercuryUtils.modalisedAtomToString(mf));
				result.add(mf);
			}
		}
		return result;
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

	public static LinkedList<EpistemicObject> proofToEpistemicObjects(IdentifierGenerator idGen, String attribAgent, MarkedQuery[] proof) {

		LinkedList<EpistemicObject> results = new LinkedList<EpistemicObject>();

		ModalisedAtom[] imfs = ProofUtils.filterStripByModalityPrefix(
				ProofUtils.stripMarking(ProofUtils.filterAssumed(proof)),
				new Modality[] {Modality.Intention});

		String s = "";
		for (int i = 0; i < imfs.length; i++) {
			s += "\n\t" + MercuryUtils.modalisedAtomToString(imfs[i]);
		}
		log("looking for intentions in" + s);

		// * generate ids via assumability function (before proving)
		// * store intentions in a dictionary, indexed by their id from proof,
		//   if a new one is detected, add to dictionary (this can be done in a loop)

		HashMap<String, IntentionalContent> rIts = new HashMap<String, IntentionalContent>();

		// TODO: make a dedicated class
		for (ModalisedAtom ma : Arrays.asList(imfs)) {
			if (ma.a.predSym.equals(agentPredSym)) {
				// agent(ID, AGENT)
//				log("  adding agent");
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
				// pre(ID, CONTENT)
//				log("  adding pre");
				FunctionTerm idTerm = (FunctionTerm) ma.a.args[0];
				FunctionTerm epstTerm = (FunctionTerm) ma.a.args[1];
				Term contentTerm = ma.a.args[2];
				if (!rIts.containsKey(idTerm.functor)) {

					rIts.put(idTerm.functor, newIntentionalContent());
				}
				IntentionalContent itc = rIts.get(idTerm.functor);
				dBelief b = termsToBelief(idGen, epstTerm, contentTerm);
				results.add(b);
				dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagement.beliefLinkModality, BeliefFormulaFactory.newElementaryFormula(b.id));
				itc.preconditions = combineDFormulas(itc.preconditions, refF);
			}
			else if (ma.a.predSym.equals(postconditionPredSym)) {
				// post(ID, CONTENT)
//				log("  adding post");
				FunctionTerm idTerm = (FunctionTerm) ma.a.args[0];
				FunctionTerm epstTerm = (FunctionTerm) ma.a.args[1];
				Term contentTerm = ma.a.args[2];
				if (!rIts.containsKey(idTerm.functor)) {

					rIts.put(idTerm.functor, newIntentionalContent());
				}
				IntentionalContent itc = rIts.get(idTerm.functor);
				dBelief b = termsToBelief(idGen, epstTerm, contentTerm);
				results.add(b);
				dFormula refF = BeliefFormulaFactory.newModalFormula(IntentionManagement.beliefLinkModality, BeliefFormulaFactory.newElementaryFormula(b.id));
				itc.postconditions = combineDFormulas(itc.postconditions, refF);
			}
		}

		Iterator<String> iter = rIts.keySet().iterator();
		while (iter.hasNext()) {
			Intention it = new Intention();
			it.id = idGen.newIdentifier();
//			it.id = IntentionManagement.counter.inc("intention");
			it.frame = new AbstractFrame();
			AttributedEpistemicStatus epst = new AttributedEpistemicStatus();
			epst.agent = IntentionManagement.thisAgent;
			epst.attribagents = new LinkedList<String>();
			epst.attribagents.add(attribAgent);
			it.estatus = epst;
			it.content = new LinkedList<IntentionalContent>();
			IntentionalContent itc = rIts.get(iter.next());
			it.content.add(itc);
			if (itc.preconditions != null && itc.postconditions != null && itc.agents != null) {
				results.add(it);
			}
			else {
				log("discarding [" + it.id + "]: incomplete");
			}
		}

		return results;
	}

	private static dBelief termsToBelief(IdentifierGenerator idGen, FunctionTerm epstT, Term contentT) {
		EpistemicStatus epst = termToEpistemicStatus(epstT);
		dFormula content = termToContent((FunctionTerm)contentT);
		return newLogicalBelief(idGen.newIdentifier(), epst, content);
//		return newLogicalBelief(IntentionManagement.counter.inc("belief"), epst, content);
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

	private static dBelief newLogicalBelief(String id, EpistemicStatus epst, dFormula content) {
		dBelief b = new dBelief();
		b.frame = new AbstractFrame();
		b.id = id;
		b.estatus = epst;
		b.type = "fact";
		b.hist = new AbstractBeliefHistory();

		FormulaProbPair log = new FormulaProbPair();
		log.prob = 1.0f;
		log.val = content;  // XXX

		FormulaValues fv = new FormulaValues();
		fv.values = new LinkedList<FormulaProbPair>();
		fv.values.add(log);

		BasicProbDistribution pd = new BasicProbDistribution();
		pd.key = counter.inc("log");
		pd.values = fv;

		b.content = pd;

		return b;
	}

	private static FunctionTerm epistemicStatusToTerm(EpistemicStatus epst) {
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

	// TODO: factor this out -- where is the API for intentions?!?
	private static IntentionalContent newIntentionalContent() {
		IntentionalContent itc = new IntentionalContent();
		itc.probValue = 1.0f;
		return itc;
	}

	private static dFormula termToContent(FunctionTerm t) {
		if (t.functor.equals("rpv")) {
			ModalFormula rF = BeliefFormulaFactory.newModalFormula(IntentionManagement.refModality, BeliefFormulaFactory.newElementaryFormula(((FunctionTerm)t.args[0]).functor));
			ModalFormula pvF = BeliefFormulaFactory.newModalFormula(((FunctionTerm)t.args[1]).functor, termToContentFormula((FunctionTerm)t.args[2]));
			return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, rF, pvF);
		}
		return null;
	}

	private static dFormula termToContentFormula(FunctionTerm t) {
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

	private static dFormula combineDFormulas(dFormula oldF, dFormula newF) {
		if (oldF == null) {
			return newF;
		}
		if (newF == null) {
			return oldF;
		}
		return BeliefFormulaFactory.newComplexFormula(BinaryOp.conj, oldF, newF);
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
					if (m1.op.equals(IntentionManagement.refModality) && m1.form instanceof ElementaryFormula) {
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

	private static void log(String str) {
		if (logging)
			System.out.println("\033[32m[Conversion]\t" + str + "\033[0m");
	}

}