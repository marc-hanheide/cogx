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

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/** 
	The class <b>LocalIdentifier</b> defines a data structure for 
	keeping track of the information for a local variable identifier. 

	@started 071106
	@version 071106
	@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 


public class LocalIdentifier {

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	public final String NO_ID = "";
	public final String NO_ADDRESS = "";


	/** The (unique) name of the variable identifier*/ 
	public String _varId; 

	/** The address of the proxy this variable refers to */ 
	public String _proxyAddress;
	
	/** The identifier in the global namespace this variable refers to */
	public String _globalIdentifier;
	
	/** The name of the local name space in which the identifier is defined*/ 
	public String _spaceId;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================	

	public LocalIdentifier () { 
		init();
	} // end constructor

	public LocalIdentifier (String v, String p, String s) { 
		init();
		_varId = v;
		_proxyAddress = p;
		_spaceId = s;
	} // end constructor

	public LocalIdentifier (String v, String p, String g, String s) { 
		init();
		_varId = v;
		_proxyAddress = p;
		_globalIdentifier = g;
		_spaceId = s;
	} // end constructor

	private void init() { 
		_varId = NO_ID;
		_proxyAddress = NO_ADDRESS;
		_globalIdentifier = NO_ID;
		_spaceId = NO_ID;
	} // end init



} // end class
