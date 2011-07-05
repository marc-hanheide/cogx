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
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryPointer;
import cast.core.CASTUtils;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;

/**
 * The <tt>Formula</tt> class implements the basic structure for building up
 * content.
 * 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de), Marc Hanheide (marc@hanheide.de)
 * @started 100521
 * @version 100521
 */
public class WMPointer extends GenericFormula<PointerFormula> {

	public static WMPointer create(Ice.Object pd) {
		return new WMPointer(pd);
	}

//	public static WMPointer create(WorkingMemoryAddress v) {
//		return new WMPointer(new PointerFormula(-1, v, ""));
//	}

	public static WMPointer create(WorkingMemoryAddress v, String type) {
		return new WMPointer(new PointerFormula(-1, v, type));
	}
	
	public static WMPointer create(WorkingMemoryPointer ptr) {
		return new WMPointer(new PointerFormula(-1, ptr.address, ptr.type));
	}
	
	public void setType(String t) {
		_content.type=t;
	}

	public String getType() {
		return _content.type;
	}

	/**
	 * Object is created from underlying slice-based datastructure, and a given
	 * probability.
	 */
	protected WMPointer(Ice.Object formula) {
		super(PointerFormula.class, formula);

	} // end constructor

	public WorkingMemoryAddress getVal() {
		return _content.pointer;
	}

	public void setVal(WorkingMemoryAddress adr) {
		_content.pointer = adr;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see de.dfki.lt.tr.beliefs.data.genericproxies.GenericFormula#toString()
	 */
	@Override
	public String toString() {
		return CASTUtils.toString(_content.pointer);
	}

} // end class
