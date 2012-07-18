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

package de.dfki.lt.tr.dialogue.util;

import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.NormalValues;
import de.dfki.lt.tr.beliefs.slice.epstatus.AttributedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.PrivateEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.intentions.Intention;
import de.dfki.lt.tr.beliefs.slice.intentions.IntentionalContent;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnderspecifiedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import de.dfki.lt.tr.dialogue.interpret.IntentionManagementConstants;
import java.util.Iterator;
import java.util.LinkedList;

public abstract class BeliefIntentionUtils {

	/**
	 * Convert an epistemic status to a string.
	 *
	 * @param epst
	 * @return string representation of epst
	 */
	public static String epistemicStatusToString(EpistemicStatus epst) {
		if (epst instanceof PrivateEpistemicStatus) {
			PrivateEpistemicStatus p = (PrivateEpistemicStatus)epst;
			return p.agent;
		}
		if (epst instanceof AttributedEpistemicStatus) {
			AttributedEpistemicStatus a = (AttributedEpistemicStatus)epst;
			String s = a.agent + "[";
			Iterator<String> i = a.attribagents.iterator();
			while (i.hasNext()) {
				s += i.next();
				if (i.hasNext()) s += ",";
			}
			s += "]";
			return s;
		}
		if (epst instanceof SharedEpistemicStatus) {
			SharedEpistemicStatus sh = (SharedEpistemicStatus)epst;
			String s = "{";
			Iterator<String> i = sh.cgagents.iterator();
			while (i.hasNext()) {
				s += i.next();
				if (i.hasNext()) s += ",";
			}
			s += "}";
			return s;
		}
		return "?";
	}

	/**
	 * Return a string representation of an intention.
	 *
	 * @param it
	 * @return the string representation of it
	 */
	public static String intentionToString(Intention it) {
		String s = "";

		s += "B{" + epistemicStatusToString(it.estatus) + "}:\n";

		for (IntentionalContent itc : it.content) {
			s += "\tI(";
			for (Iterator<String> i = itc.agents.iterator(); i.hasNext(); ) {
				s += i.next();
				if (i.hasNext()) { s += ","; }
			}
			s += "):\n";
			s += "\t\t <Pre>" + dFormulaToString(itc.preconditions) + " ^\n";
			s += "\t\t<Post>" + dFormulaToString(itc.postconditions) + "\n\t\t@ p=" + itc.probValue + "\n";
		}

		return s;
	}

	/**
	 * Return a string representation of a simple belief.
	 *
	 * @param b
	 * @return the string representation of b
	 */
	public static String logicalBeliefToString(dBelief b) {
		String s = "";
		s += "\tB{" + epistemicStatusToString(b.estatus) + "}:";

		if (b.content instanceof BasicProbDistribution) {
			BasicProbDistribution pd = (BasicProbDistribution)b.content;
			if (pd.values instanceof FormulaValues) {
				FormulaValues fv = (FormulaValues)pd.values;
				Iterator<FormulaProbPair> i = fv.values.iterator();
				while (i.hasNext()) {
					FormulaProbPair fp = i.next();
					s += "\n\t\t" + dFormulaToString(fp.val) + " @ p=" + fp.prob;
				}
				return s;
			}
		}
		return null;
	}

	public static String beliefToString(dBelief b) {
		String s = "B{" + epistemicStatusToString(b.estatus) + "}:\n";
		String cs = probDistributionToString(b.content);
		if (cs != null) {
			s += cs;
		}
		return s;
	}

	public static String probDistributionToString(ProbDistribution pd) {
		if (pd instanceof BasicProbDistribution) {
			return basicProbDistributionToString((BasicProbDistribution)pd);
		}
		else if(pd instanceof CondIndependentDistribs) {
			return condIndependentDistribsToString((CondIndependentDistribs)pd);
		}
		else {
			return "UNKNOWN-PROB-DISTRIB";
		}
	}

	public static String basicProbDistributionToString(BasicProbDistribution bpd) {
		return "(basic '" + bpd.key + "' " + distributionValuesToString(bpd.values) + ")";
	}
	
	public static String condIndependentDistribsToString(CondIndependentDistribs cids) {
		String s = "(condind";
		s += "\n";
		for (String key : cids.distribs.keySet()) {
			s += "\t'" + key + "' -> " + probDistributionToString(cids.distribs.get(key));
			s += "\n";
		}
		s += ")";
		return s;
	}

	public static String distributionValuesToString(DistributionValues dv) {
		if (dv instanceof FormulaValues) {
			return formulaValuesToString((FormulaValues) dv);
		}
		else if (dv instanceof NormalValues) {
			return "NORMAL-VALUES";
		}
		else {
			return "UNKNOWN-DISTRIB-VALUES";
		}
	}

	public static String formulaValuesToString(FormulaValues fvs) {
		String s = "{";
		for (FormulaProbPair fpp : fvs.values) {
			s += "(" + dFormulaToString(fpp.val) + " @ p=" + fpp.prob + ") ";
		}
		s += "}";
		return s;
	}

	/**
	 * Return a string representation of a formula.
	 *
	 * @param f
	 * @return the string representation of f
	 */
	public static String dFormulaToString(dFormula f) {
		String s = "";
		if (f instanceof ComplexFormula) {
			ComplexFormula cplxF = (ComplexFormula) f;
			s += "(";
			for (Iterator<dFormula> i = cplxF.forms.iterator(); i.hasNext(); ) {
				dFormula df = i.next();
				s += dFormulaToString(df);
				if (i.hasNext()) {
					s += cplxF.op == BinaryOp.conj ? " ^ " : " | ";
				}
			}
			s += ")";
			return s;
		}
		if (f instanceof NegatedFormula) {
			NegatedFormula nF = (NegatedFormula) f;
			s += "not(" + dFormulaToString(nF.negForm) + ")";
			return s;
		}
		if (f instanceof ElementaryFormula) {
			ElementaryFormula eF = (ElementaryFormula) f;
			s += eF.prop;
			return s;
		}
		if (f instanceof ModalFormula) {
			ModalFormula mF = (ModalFormula) f;
			s += "<" + mF.op + ">" + dFormulaToString(mF.form);
			return s;
		}
		if (f instanceof PointerFormula) {
			PointerFormula pF = (PointerFormula) f;
			s += "[" + pF.pointer.subarchitecture + "," + pF.pointer.id + "]";
			return s;
		}
		if (f instanceof UnderspecifiedFormula) {
			UnderspecifiedFormula uF = (UnderspecifiedFormula) f;
			s += "?" + uF.arglabel;
			return s;
		}
		return "DFORMULA";
	}

	/**
	 * Collect belief identifiers in the given intention.
	 *
	 * @param it
	 * @return list of belief identifiers
	 */
	public static LinkedList<String> collectBeliefIdsInIntention(Intention it) {
		LinkedList<String> eos = new LinkedList<String>();

		for (IntentionalContent itc : it.content) {
			eos.addAll(collectBeliefIdsInDFormula(itc.preconditions));
			eos.addAll(collectBeliefIdsInDFormula(itc.postconditions));
		}
		return eos;
	}

	/**
	 * Collect belief identifiers in the given logical formula.
	 * @param f
	 * @return list of belief identifiers
	 */
	public static LinkedList<String> collectBeliefIdsInDFormula(dFormula f) {
		LinkedList<String> eos = new LinkedList<String>();
		if (f instanceof ComplexFormula) {
			ComplexFormula cplxF = (ComplexFormula)f;
			for (dFormula df : cplxF.forms) {
				eos.addAll(collectBeliefIdsInDFormula(df));
			}
		}
		else if (f instanceof ModalFormula) {
			ModalFormula mF = (ModalFormula)f;
			if (mF.op.equals(IntentionManagementConstants.beliefLinkModality)) {
				if (mF.form instanceof ElementaryFormula) {
					ElementaryFormula eF = (ElementaryFormula)mF.form;
					eos.add(eF.prop);
				}
			}
		}
		return eos;
	}

	public static boolean isRobotsIntention(Intention it) {
		if (it.content.size() == 1) {
			IntentionalContent itnc = it.content.get(0);
			if (itnc.agents.size() == 1 && itnc.agents.get(0).equals(IntentionManagementConstants.thisAgent)) {
				return true;
			}
		}
		return false;
	}

	public static boolean isHumansIntention(Intention it) {
		if (it.content.size() == 1) {
			IntentionalContent itnc = it.content.get(0);
			if (itnc.agents.size() == 1 && itnc.agents.get(0).equals(IntentionManagementConstants.humanAgent)) {
				return true;
			}
		}
		return false;
	}

}
