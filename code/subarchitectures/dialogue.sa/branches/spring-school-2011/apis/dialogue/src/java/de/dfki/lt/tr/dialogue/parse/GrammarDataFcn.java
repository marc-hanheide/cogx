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

import de.dfki.lt.tr.dialogue.util.DialogueException;

//-----------------------------------------------------------------
//JAVA IMPORTS
//-----------------------------------------------------------------
import java.util.Iterator;
import java.util.Vector;

/**
The abstract class <b>GrammarDataFcn</b> implements the basic 
functionality for the GrammarData interface. The data is internally 
stored (as <b>Object</b>s) on a Vector <tt>a_data</tt>, which 
is accessed through an iterator. 

@version 070813
@since   070813
@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/

public class GrammarDataFcn 
implements GrammarData
{

	// =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================	
	
	// A list of Object data items
	private Vector<Object> a_data; 

	// =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================	

	public GrammarDataFcn () { 
		init (); 
	} // end GrammarDataFcn

	private void init () { 
		a_data = new Vector<Object>();
	} // end init

	// =================================================================
    // ACCESS METHODS
    // =================================================================	

	/** Adds the provided data item as object to the list of grammar data 
		
		@param di The data item to be stored as part of the results
	*/

	public void addDataItem (Object di) { 
		a_data.addElement(di);
	} // end addDataItem

	/** Returns the results as an iterator over the internally stored Vector 
	
		@return Iterator The iterator over the Vector with data items (of type Object)
	*/
	
	public Iterator getData () { 
		return a_data.iterator(); 
	} // end getData  


	
} // end class
