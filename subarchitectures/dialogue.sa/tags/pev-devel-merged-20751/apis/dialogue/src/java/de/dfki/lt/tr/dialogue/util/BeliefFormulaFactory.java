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

import cast.cdl.WorkingMemoryAddress;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BinaryOp;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ComplexFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ModalFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.NegatedFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import java.util.Arrays;
import java.util.List;
import java.util.LinkedList;

public abstract class BeliefFormulaFactory {

	public static ModalFormula newModalFormula(String modality, dFormula f) {
		ModalFormula modF = new ModalFormula();
		modF.id = -1;  // FIXME: generate a unique id
		modF.op = modality;
		modF.form = f;
		return modF;
	}

	public static NegatedFormula newNegatedFormula(dFormula f) {
		NegatedFormula negF = new NegatedFormula();
		negF.id = -1;  // FIXME: generate a unique id
		negF.negForm = f;
		return negF;
	}

	public static ElementaryFormula newElementaryFormula(String prop) {
		ElementaryFormula elemF = new ElementaryFormula();
		elemF.id = -1;  // FIXME: generate a unique id
		elemF.prop = prop;
		return elemF;
	}

	public static ComplexFormula newComplexFormula(BinaryOp op, dFormula f1, dFormula f2) {
		ComplexFormula cplxF = new ComplexFormula();
		cplxF.id = -1;  // FIXME: generate a unique id
		cplxF.op = op;
		cplxF.forms = new LinkedList<dFormula>();
		cplxF.forms.add(f1);
		cplxF.forms.add(f2);
		return cplxF;
	}

	public static ComplexFormula newComplexFormula(BinaryOp op, dFormula[] fs) {
		ComplexFormula cplxF = new ComplexFormula();
		cplxF.id = -1;  // FIXME: generate a unique id
		cplxF.op = op;
		cplxF.forms = new LinkedList<dFormula>();
		cplxF.forms.addAll(Arrays.asList(fs));
		return cplxF;
	}

	public static ComplexFormula newComplexFormula(BinaryOp op, List<dFormula> fs) {
		ComplexFormula cplxF = new ComplexFormula();
		cplxF.id = -1;  // FIXME: generate a unique id
		cplxF.op = op;
		cplxF.forms = new LinkedList<dFormula>();
		cplxF.forms.addAll(fs);
		return cplxF;
	}

	public static PointerFormula newPointerFormula(WorkingMemoryAddress wma) {
		PointerFormula pF = new PointerFormula();
		pF.pointer = wma;
		return pF;
	}

	public static dFormula maybeNegatedElementaryFormula(String proposition, boolean isNegated) {
		ElementaryFormula elem = new ElementaryFormula(0, proposition);
		if (isNegated) {
			return new NegatedFormula(0, elem);
		}
		else {
			return elem;
		}
	}

}
