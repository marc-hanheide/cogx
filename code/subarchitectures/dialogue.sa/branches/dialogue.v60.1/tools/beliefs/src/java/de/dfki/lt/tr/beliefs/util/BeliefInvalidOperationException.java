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
package de.dfki.lt.tr.beliefs.util;

/**
 * Exception type to deal with runtime-catchable errors resulting from invalid operations on a data structure in the Belief API
 * 
 * @author gj
 * @version 100521
 */

public class BeliefInvalidOperationException 
extends BeliefException 
{

	/**
	 * 
	 */
	private static final long serialVersionUID = -466767075264262816L;

	public BeliefInvalidOperationException (String msg) 
	{ 
		super(msg); 
	} // end constructor
	
} // end class
