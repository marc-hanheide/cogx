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

//-----------------------------------------------------------------
// BINDING IMPORTS
//-----------------------------------------------------------------	
import binding.BindingException;

//-----------------------------------------------------------------
// JAVA IMPORTS
//-----------------------------------------------------------------	
import java.util.Hashtable;
import java.util.Iterator;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/** 
	The class <b>LocalRelation</b> provides a data structure for 
	proxy relations. The class stores information about the proxy
	address, the mode of the relation, and between which proxies
	the relation holds ("from" and "to"). For the latter we are
	using proxy addresses rather than identifiers for proxy addresses.
	There are two reasons. For one, a monitor state can easily retrieve 
	identifiers in either a local or the global name space on the basis 
	of these addresses. Secondly, proxy relations can in principle also 
	point to other relations. 
	
	@started 071106
	@version 071106
	@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 


public class ProxyRelation {

	//=================================================================
	// CLASS-GLOBAL DATA STRUCTURES
	//=================================================================
	
	public final String NO_ADDRESS = "";
	
	public final String NO_MODE = "";
	
	public String _proxyAddress;
	public String _proxyMode;
	public String _fromProxyAddress;
	public String _toProxyAddress;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public ProxyRelation () { 
		init();
	} // end constructor
	
	public ProxyRelation (String pa, String pm, String fpa, String tpa) { 
		init();
		_proxyAddress = pa;
		_proxyMode    = pm;
		_fromProxyAddress = fpa;
		_toProxyAddress = tpa;		
	} // end constructor
	
	private void init () {
		_proxyAddress = NO_ADDRESS;
		_proxyMode    = NO_MODE;
		_fromProxyAddress = NO_ADDRESS;
		_toProxyAddress = NO_ADDRESS;
	} // end init


} // end class
