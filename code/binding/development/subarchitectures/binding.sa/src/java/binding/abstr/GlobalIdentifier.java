//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package binding.abstr;

//=================================================================
// IMPORTS
//=================================================================

import java.util.Vector; 

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/** 
	The class <b>GlobalIdentifier</b> defines a data structure for 
	keeping track of the information for a global variable identifier. 

	@started 071106
	@version 071106
	@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 


public class GlobalIdentifier {

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	public final String NO_ADDRESS = "";

	/** The (unique) name of the variable identifier*/ 
	public String _varId; 

	/** The address of the proxy this variable refers to */ 
	public String _proxyAddress;
	
	/** The identifiers in local namespaces this variable refers to */
	public Vector<LocalIdentifier> _localIdentifiers;
	
	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================
	
	public GlobalIdentifier () { 
		init();
	} // end constructor
				
	protected void init () { 
		_varId = "";
		_proxyAddress = NO_ADDRESS;
		_localIdentifiers = new Vector<LocalIdentifier>();
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** 
		The method <i>addLocalIdentifier</i> updates the list with local
		identifiers with the given local identifiers. 
		
		@param locVar The local identifier to be added
	*/ 

	public void addLocalIdentifier (LocalIdentifier locVar) { 
		_localIdentifiers.addElement(locVar);
	} // end addLocalIdentifier

	
	



} // end class
