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
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;
import java.util.Iterator;
import java.util.LinkedList;

public abstract class IntentionUtils {

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

		s += "\tB{" + epistemicStatusToString(it.estatus) + "}:\n";

		for (IntentionalContent itc : it.content) {
			s += "\t\tI(";
			for (Iterator<String> i = itc.agents.iterator(); i.hasNext(); ) {
				s += i.next();
				if (i.hasNext()) { s += ","; }
			}
			s += "):\n";
			s += "\t\t\t <Pre>" + dFormulaToString(itc.preconditions) + " ^\n";
			s += "\t\t\t<Post>" + dFormulaToString(itc.postconditions) + "\n\t\t@ p=" + itc.probValue + "\n";
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

	/**
	 * Return a string representation of a formula.
	 *
	 * @param f
	 * @return the string representation of f
	 */
	private static String dFormulaToString(dFormula f) {
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
			if (mF.op.equals(IntentionManagement.beliefLinkModality)) {
				if (mF.form instanceof ElementaryFormula) {
					ElementaryFormula eF = (ElementaryFormula)mF.form;
					eos.add(eF.prop);
				}
			}
		}
		return eos;
	}


}
