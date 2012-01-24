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
package de.dfki.lt.tr.beliefs.data.formulas;

// Belief API slice
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;

/**
 * The <tt>Formula</tt> class implements the basic structure for building up
 * content.
 * 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de), Marc Hanheide (marc@hanheide.de)
 * @started 100521
 * @version 100521
 */
public class BoolFormula extends GenericFormula<BooleanFormula> {

	public static BoolFormula create(Ice.Object pd) {
		return new BoolFormula(pd);
	}
	
	public static BoolFormula create(boolean v) {
		return new BoolFormula(new BooleanFormula(-1, v));
	}
	
	

	/**
	 * Object is created from underlying slice-based datastructure, and a given
	 * probability.
	 */
	protected BoolFormula(Ice.Object formula) {
		super(BooleanFormula.class, formula);

	} // end constructor

	public boolean getVal() {
		return _content.val;
	}

	
} // end class
