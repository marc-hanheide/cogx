//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General License for more details.
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
import java.util.Iterator;

import binding.BindingException;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/**
	The interface <b>MonitorState</b> defines methods which
	any binding monitor state should provide for querying information 
	about local- and global namespace Identifiers to proxies, (of
	whatever proxy type). 

	A binding monitor state is essentially a collection of maps, for 
	maintaining mappings between identifiers for nodes in graphs
	modelling content (e.g. maps, logical forms, ...), and proxies on
	binding working memory representing (access to) that content. For 
	graph node identifiers, we distinguish a local and a global namespace. 

	
	<ul> 
	<li> Mappings between local identifiers and proxies
	<li> Mappings between global identifiers and local identifiers
	<li> Mappings global identifiers and proxies
	<li> Mappings between relation proxies, and local- and global identifiers
	</ul> 

	@started 071106
	@version 071106
	@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// JAVA 
//=================================================================

interface MonitorState {

	//=================================================================
	// ACCESSOR METHODS 
	//=================================================================
	
	//-----------------------------------------------------------------
	// GLOBAL NAME SPACE ACCESS
	//-----------------------------------------------------------------	

	/**
		The method <i>existsGlobalSpaceProxyAddress</i> returns a boolean indicating
		whether there is a proxy address for the given identifier in the global name space. 
		
		@param  varId    The identifier in the global name space
		@return String	 The proxy referred to by the identifier in the global name space
	*/ 
	
	public boolean existsGlobalSpaceProxyAddress (String varId); 
	
	/**
		The method <i>getGlobalSpaceProxyAddress</i> returns a proxy address 
		for the given identifier in the global name space. 
		
		@param  varId    The identifier in the global name space
		@return String	 The proxy referred to by the identifier in the global name space
		@throws BindingException If the global identifier is unknown
	*/ 
	
	String getGlobalSpaceProxyAddress (String varId)
		throws BindingException; 



	/**
		The method <i>getGlobalSpaceIdentifier</i> returns  an identifier 
		in the global name space, for the given reference. 
		
		@param  varId The identifier of the proxy 
		@return GlobalIdentifier	 The variable identifier in the global name space 
	*/ 
	
	GlobalIdentifier getGlobalSpaceIdentifier (String varId)
		throws BindingException; 



	/**
		The method <i>getGlobalSpaceIdentifierReference</i> returns a reference to an identifier 
		in the global name space, for the given proxy address. 
		
		@param proxyAddr The identifier of the proxy 
		@return String	 The variable identifier in the global name space referring to the given proxy
	*/ 
	
	String getGlobalSpaceIdentifierReference (String proxyAddr); 


	/**
		The method <i>getGlobalSpaceIdentifiers</i> returns an Iterator over identifiers
		in the global name space
		
		@return Iterator	 The iterator over variable identifiers in the global name space, referring to proxies
	*/ 
	
	Iterator getGlobalSpaceIdentifiers (); 

	//-----------------------------------------------------------------
	// LOCAL NAME SPACE ACCESS
	//-----------------------------------------------------------------	
	
	/**
		The method <i>addLocalSpaceIdentifier</i> adds a Identifier to the
		given proxy in the given namespace
		
		@param varId	The variable identifier used as Identifier
		@param spaceId	The local namespace
		@param proxyAddr The proxy referred to (by address)
		@throws BindingException If the Identifier cannot be added (usually, varId already exists)
		
	*/

	void addLocalSpaceIdentifier (String varId, String spaceId, String proxyAddr)
		throws BindingException; 


	/**
		The method <i>addLocalToGlobalIdentifier</i> links a given local identifier, in the 
		given local namespace, to the given identifier in the global namespace. 
		
		@param	varId	The variable identifier in the given local namespace
		@param	spaceId	The local namespace
		@param	globalVarId	The identifier in the global namespace
		@throws BindingException If the Identifier cannot be added (usually, a link already exists)
	*/
	
	void addLocalToGlobalIdentifier (String varId, String spaceId, String globalVarId)
		throws BindingException; 
		
	/** 
		The method <i>excludeIdentifier</i> puts the given variable identifier on a list of 
		identifiers which should not be considered for (further) processing. 
		
		@param varId	The variable identifier to be excluded
		@param spaceId	The local name space
	*/ 

	void excludeIdentifier (String varId, String spaceId);

	/** 
		The method <i>excludeIdentifier</i> puts each of the given variable identifiers on the
		iterator onto a list of identifiers which should not be considered for (further) processing. 
		
		@param varId	The variable identifier to be excluded
		@param spaceId	The local name space
	*/ 

	public void excludeIdentifiers (Iterator varsIter, String spaceId);

	/** 
		The method <i>existsLocalSpace</i> returns a boolean to indicate whether 
		there is a local namespace with the given identifier
		
		@param spaceId	The identifier of the local namespace
		@return boolean Whether there is a local space with that identifier
	*/ 

	public boolean existsLocalSpace (String spaceId); 

	/** 
		The method <i>existsLocalSpaceProxyAddress</i> returns a boolean to indicate whether 
		for the given variable identifier in the given local namespace, there is a known proxy
		address the variable refers to. 
		
		@param varId	The variable identifier in the local namespace
		@param spaceId	The identifier of the local namespace
		@return boolean Whether there is a known proxy address which the variable refers to
	*/ 

	boolean existsLocalSpaceProxyAddress (String varId, String spaceId); 

	/** 
		The method <i>existsLocalSpaceProxyRelation</i> returns a boolean indicating whether 
		in the given local name space, a proxy relation with the (template) information as
		provided by the given relation already exists. 
	*/ 

	public boolean existsLocalSpaceProxyRelation (String spaceId, ProxyRelation rel) 
		throws BindingException; 	

	/**
		The method <i>getLocalSpaceProxyAddress</i> returns a proxy address 
		for the given identifier in the given local name space. 
		
		@param  varId    The identifier in the local name space
		@param  spaceId	 The identifier of the local name space
		@return String	 The proxy referred to by the identifier in the global name space
	*/ 
	
	String getLocalSpaceProxyAddress (String varId, String spaceId) 
		throws BindingException; 	
	
	
	/**
		The method <i>getLocalSpaceIdentifier</i> returns a reference to an identifier 
		in the given local name space, for the given proxy address. 
		
		@param spaceId	The identifier of the local name space
		@param proxyAddr The identifier of the proxy 
		@return String	 The variable identifier in the local name space referring to the given proxy
	*/ 
	
	String getLocalSpaceIdentifierReference (String spaceId, String proxyAddr)
		throws BindingException; 	
	
	/**
		The method <i>getLocalSpaceIdentifiers</i> returns an Iterator over identifiers 
		in the given local name space 
		
		@param spaceId	The identifier of the local name space
		@return Iterator The iterator over variable identifiers in the local name space
	*/ 
	
	Iterator getLocalSpaceIdentifiers (String spaceId);

	/**
		The method <i>getLocalSpaceProxyRelations</i> returns an Iterator over identifiers 
		for proxy relations in the given local name space in which the given variable
		identifier is involved
		
		@param  varId	The variable identifier
		@param  spaceId	The identifier of the local name space
		@return Iterator The iterator over variable identifiers in the local name space
	*/ 
	
	public Iterator getLocalSpaceProxyRelations (String varId, String spaceId) 
		throws BindingException;  

	/** 
		The method <i>isExcluded</i> returns a boolean to indicate whether 
		for the given variable identifier in the given local namespace, it is known 
		that the variable has been excluded from further processing
						
		@param varId	The variable identifier in the local namespace
		@param spaceId	The identifier of the local namespace
		@return boolean Whether the given identifier has been excluded from further processing
	*/ 

	boolean isExcluded (String varId, String spaceId);


	/**
		The method <i>removeLocalSpaceIdentifier</i> removes the variable identifier from 
		the given local name space. This includes removing links to proxy addresses, and 
		identifiers in the global name space. 

		@param varId	The variable identifier to be removed
		@param spaceId	The local name space
	*/ 

	void removeLocalSpaceIdentifier (String varId, String spaceId) throws BindingException;

	/**
		The method <i>removeLocalSpaceProxyRelation</i> removes the proxy relation from 
		the given local name space. 

		@param relLabel	The proxy relation to be removed
		@param spaceId	The local name space
		@throws BindingException If the local name space is unknown		
	*/ 

	public void removeLocalSpaceProxyRelation (String relLabel, String spaceId) throws BindingException; 

	/**
		The method <i>updateLocalSpaceIdentifier</i> updates an existing Identifier in the given namespace
		
		@param varId	The variable identifier used as Identifier
		@param spaceId	The local namespace
		@param proxyAddr The proxy referred to (by address)
		@throws BindingException If the identifier cannot be added, or the local name space is unknown
		
	*/

	public void updateLocalSpaceIdentifier (String varId, String spaceId, String proxyAddr)
		throws BindingException;



} // end interface
