// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.beliefs.data.genericproxies;

// Belief API slice
import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidQueryException;

/**
 * The <tt>Formula</tt> class implements the basic structure for building up
 * content.
 * 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de), Marc Hanheide (marc@hanheide.de)
 * @started 100521
 * @version 100521
 */
public class GenericFormula<T extends dFormula> extends Proxy<T> {

	public static <T2 extends dFormula> GenericFormula<T2> create(
			Class<? extends T2> type, Ice.Object pd) {
		return new GenericFormula<T2>(type, pd);
	}

	/**
	 * Object is created from underlying slice-based datastructure, and a given
	 * probability.
	 */
	protected GenericFormula(Class<? extends T> type, Ice.Object formula) {
		super(type, formula);

	} // end constructor

	/**
	 * Returns the identifier of the formula. By default this is set to -1, upon
	 * initialization.
	 * 
	 * @return int The identifier of the formula
	 * @throws BeliefNotInitializedException
	 *             If the formula has not been initialized
	 */

	public int getId() {
		return _content.id;
	} // end getId

	/**
	 * Returns the proposition for a formula, provided the formula is
	 * propositional
	 * 
	 * @return String The proposition of the formula
	 * @throws BeliefNotInitializedException
	 *             If the formula is not initialized
	 * @throws BeliefInvalidQueryException
	 *             If the formula is not of type proposition
	 */

	public String getProposition() throws BeliefInvalidQueryException {
		if (!(_content instanceof ElementaryFormula)) {
			throw new BeliefInvalidQueryException(
					"Cannot query [proposition] for formula: Formula not of type proposition");
		} else {
			return ((ElementaryFormula) _content).prop;
		}
	} // end getProposition

	/**
	 * Returns the integer for a formula, provided the formula is an integer
	 * 
	 * @return String The proposition of the formula
	 * @throws BeliefNotInitializedException
	 *             If the formula is not initialized
	 * @throws BeliefInvalidQueryException
	 *             If the formula is not of type proposition
	 */

	public int getInteger() throws BeliefInvalidQueryException {
		if (!(_content instanceof IntegerFormula)) {
			throw new BeliefInvalidQueryException(
					"Cannot query [proposition] for formula: Formula not of type proposition");
		} else {
			return ((IntegerFormula) _content).val;
		}
	}

	public double getDouble() {
		if (!(_content instanceof FloatFormula)) {
			throw new BeliefInvalidQueryException(
					"Cannot query [proposition] for formula: Formula not of type proposition");
		} else {
			return ((FloatFormula) _content).val;
		}
	}

	public boolean getBoolean() {
		if (!(_content instanceof BooleanFormula)) {
			throw new BeliefInvalidQueryException(
					"Cannot query [proposition] for formula: Formula not of type proposition");
		} else {
			return ((BooleanFormula) _content).val;
		}
	}

	/**
	 * Returns whether the formula is a proposition (or not)
	 * 
	 * @return boolean True if the formula is a proposition
	 * @throws BeliefNotInitializedException
	 *             If the formula has not been initialized
	 */

	public boolean isProposition() {
		return (_content instanceof ElementaryFormula);
	} // end isProposition

	/**
	 * Sets the identifier of the formula.
	 * 
	 * @param id
	 *            The identifier (represented as an integer) for the formula
	 * @throws BeliefNotInitializedException
	 *             If the formula has not been initialized
	 */

	public void setId(int id) {
		_content.id = id;
	} // end setId

	public Formula getAsFormula() {
		return Formula.create(this.get());
	}

	// /**
	// * Returns the formula and probability as a pair
	// *
	// * @return FormulaProbPair The slice-based datastructure of a formula and
	// * its associated probability
	// * @throws BeliefNotInitializedException
	// * If the formula is not instantiated
	// * @throws BeliefInvalidOperationException
	// * If the probability or the formula is not initialized
	// */
	//
	// public FormulaProbPair getAsPair() throws BeliefNotInitializedException,
	// BeliefInvalidOperationException {
	// if (_probability == -1) {
	// throw new BeliefInvalidOperationException(
	// "Cannot create formula,probability pair for non-initialized probability");
	// }
	// return new FormulaProbPair(_content, _probability);
	// } // end getAsPair
	/*
	 * (non-Javadoc)
	 * 
	 * @see de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy#toString()
	 */
	@Override
	public String toString() {
		if (_content instanceof IntegerFormula)
			return Integer.toString(((IntegerFormula) _content).val);
		else if (_content instanceof FloatFormula)
			return Double.toString(((FloatFormula) _content).val);
		else if (_content instanceof BooleanFormula)
			return Boolean.toString(((BooleanFormula) _content).val);
		else if (_content instanceof ElementaryFormula)
			return ((ElementaryFormula) _content).prop;
//		else if (_content instanceof PointerFormula)
//			return CASTUtils.toString(((PointerFormula) _content).pointer);
		return super.toString();
	}

	@Override
	public boolean equals(Object obj) {
		GenericFormula<T> ft = this;
		if (!(obj instanceof GenericFormula<?>)) {
			return super.equals(obj);
		}
		GenericFormula<?> fo = (GenericFormula<?>) obj;
		if (!ft.get().getClass().isInstance(fo.get()))
			return false;
		if (ft.get() instanceof IntegerFormula)
			return ft.getInteger() == fo.getInteger();
		if (ft.get() instanceof FloatFormula)
			return ft.getDouble() == fo.getDouble();
		if (ft.get() instanceof BooleanFormula)
			return ft.getBoolean() == fo.getBoolean();
		if (ft.get() instanceof ElementaryFormula)
			return ft.getProposition() == fo.getProposition();
		return false;
	}

} // end class
