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

//=================================================================
//IMPORTS

// Belief API slice

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.slice.framing.AbstractFrame;


/**
 * The class <tt>Frame</tt> provides access to the situated frame of a belief.
 * 
 * @author 	Geert-Jan M. Kruijff
 * @since	100523
 * @version	100523
 */

public class GenericFrame<T extends AbstractFrame> extends Proxy<T> {

	public static <T2 extends AbstractFrame> GenericFrame<T2> create(Class<? extends T2> type, AbstractFrame frame) {
		return new GenericFrame<T2>(type, frame);
	}
	
	protected GenericFrame(Class<? extends T> class1, AbstractFrame content) {
		super(class1, content);
	}
	
} // end class
