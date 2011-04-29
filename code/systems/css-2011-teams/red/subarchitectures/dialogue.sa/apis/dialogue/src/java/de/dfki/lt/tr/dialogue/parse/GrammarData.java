// =================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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
package de.dfki.lt.tr.dialogue.parse;

//=================================================================
// IMPORTS

// Java
import java.util.Iterator;

/** 
 * The interface just provides an abstraction from any particular
 * type of results a module implementing access to a grammar may yield.
 * 
 * @author 	Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100607 
*/
public interface GrammarData {

	/** Adds the provided data item as object to the list of grammar data 
	 * @param di The data item to be stored as part of the results
	 */
	public void addDataItem (Object di);  

	/** Returns the results as an iterator over the data items
	 * 	@return Iterator An iterator over the list with data items (of type Object)
	 */
	public Iterator getData (); 
	
} // end interface
