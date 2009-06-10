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
import java.util.Vector;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/**
	The  class <b>AbstractMonitorState</b> implements the methods 
	which any binding monitor state should provide for querying information 
	about local- and global namespace Identifiers to proxies, (of
	whatever proxy type). The state maintains a map of local proxy structures
	which have been generated through the monitor, indexed by global 
	identifiers. 

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
	@version 080617
	@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// JAVA 
//=================================================================


public  class AbstractMonitorState 
	implements MonitorState
{

	//=================================================================
	// CLASS-GLOBAL DATA STRUCTURES
	//=================================================================

	/** Hashtable with local name spaces: String-key, identifier of the name space; Value-LocalNameSpace, the name space*/ 
	Hashtable<String,LocalNameSpace> _localNameSpaces; 

	/** Hashtable with the global name space: String-key, identifier of the global variable; Value-GlobalIdentifier, the global variable*/ 
	Hashtable<String,GlobalIdentifier> _globalNameSpace;
	
	/** Hashtable mapping proxy addresses to global name space: String-key, proxy address; Value-String, identifier id of the global variable */ 
	Hashtable<String,String> _proxiesToGlobalNameSpace; 
	

	/** Hashtable mapping global name space identifiers to local proxy structures (stored as Object) */
	Hashtable<String,Object> _localProxyStructures;


	public final String NO_ADDRESS = "";
	public final String NO_ID = "";	
	
	boolean logging = true;
	
	//=================================================================
	// CONSTRUCTOR METHODS 
	//=================================================================
	
	public AbstractMonitorState () { 
		init();
	} // end constructor
	
	/** 
		The method <i>init</i> initializes the global data structures
	*/ 
	
	protected void init () { 
		_localNameSpaces = new Hashtable<String,LocalNameSpace>();
		_globalNameSpace = new Hashtable<String,GlobalIdentifier>();
		_proxiesToGlobalNameSpace = new Hashtable<String,String>();
		_localProxyStructures = new Hashtable<String,Object>();
	} // end init
	
	//=================================================================
	// ACCESSOR METHODS 
	
	public void setLogging (boolean l) { logging = l; }

	//-----------------------------------------------------------------
	// PROXY STRUCTURES
	//-----------------------------------------------------------------	

	/**
	The method <i>addProxyStructure</i> stores the given proxy structure
	with the global identifier as index, in the Hashtable _localProxyStructures. 

	@param	gid		The identifier to index the proxy structure by
	@param	prx		The proxy structure (as Object)
	@throws	BindingException When there is already a proxy structure with the given id
	*/ 

	public void addProxyStructure (String gid, Object prx) 
		throws BindingException 
	{ 
		if (_localProxyStructures.containsKey(gid)) { 
			throw new BindingException ("Proxy structure with global identifier ["+gid+"] already exists in monitor state -- not stored"); 
		} else {
			_localProxyStructures.put(gid,prx);
		} // end if..else check for existence
	} // end addProxyStructure

	/**
	The method <i>existsProxyStructure</i> returns a boolean indicating whether the Hashtable _localProxyStructures contains a proxy structure
	with the given identifier. 
	
	@param	gid		The identifier to check for
	@return boolean	Whether there is a proxy structure in the table with the given identifier
	*/ 

	public boolean existsProxyStructure (String gid) {	
		return _localProxyStructures.containsKey(gid);
	} // end existsProxyStructure

	/**
	The method <i>getProxyStructure</i> returns the proxy structure with the given identifier, provided there is an object with that id. 
	
	@param	gid		The identifier of the proxy structure to be obtained
	@throws BindingException When there is no proxy structure with that identifier
	*/ 


	public Object getProxyStructure (String gid) 
		throws BindingException 
	{ 
		if (_localProxyStructures.containsKey(gid)) { 
			return _localProxyStructures.get(gid);
		} else { 
			throw new BindingException ("No proxy structure found in monitor state with global identifier ["+gid+"]");
		}
	} // end getProxyStructure
	
	/**
	The method <i>updateProxyStructure</i> overwrites an existing proxy structure for the given identifier, with the 
	provided proxy structure. 
	
	@param	gid		The global identifier of the proxy structure
	@param	prx		The proxy structure
	*/

	public void updateProxyStructure (String gid, Object prx) {
		_localProxyStructures.put(gid,prx);
	} // end updateProxyStructure
	
	//-----------------------------------------------------------------
	// GLOBAL NAME SPACE ACCESS
	//-----------------------------------------------------------------	
	
	/** 
		The method <i>addGlobalSpaceIdentifier</i> adds the given global 
		identifier to the global name space, and updates the table 
		mapping proxy address to global identifiers. 
	
		@param gId	The global identifier to be added
	*/ 
	
	public void addGlobalSpaceIdentifier (GlobalIdentifier gId) 
	{ 
		log("Adding global space identifier with id ["+gId._varId+"]");
		_globalNameSpace.put(gId._varId, gId);
		_proxiesToGlobalNameSpace.put(gId._proxyAddress,gId._varId);
	}  // end addGlobalSpaceIdentifier	
	
	
	/**
		The method <i>existsGlobalSpaceProxyAddress</i> returns a boolean indicating
		whether there is a proxy address for the given identifier in the global name space. 
		
		@param  varId    The identifier in the global name space
		@return String	 The proxy referred to by the identifier in the global name space
	*/ 
	
	public boolean existsGlobalSpaceProxyAddress (String varId) 
	{  
		boolean result = false;
		if (_globalNameSpace.containsKey(varId)) { 
			GlobalIdentifier gVar = (GlobalIdentifier) _globalNameSpace.get(varId);
			String proxyAddress = gVar._proxyAddress;
			if (!proxyAddress.equals(gVar.NO_ADDRESS)) { 
				result = true;
			} // end if..check for address
		} // end if.. check for known id
		return result;
	} // end getGlobalSpaceProxyAddress

	/**
		The method <i>getGlobalSpaceProxyAddress</i> returns a proxy address 
		for the given identifier in the global name space. 
		
		@param  varId    The identifier in the global name space
		@return String	 The proxy referred to by the identifier in the global name space
	*/ 
	
	public String getGlobalSpaceProxyAddress (String varId) 
		throws BindingException
	{  
		if (_globalNameSpace.containsKey(varId)) { 
			GlobalIdentifier gVar = (GlobalIdentifier) _globalNameSpace.get(varId);
			String proxyAddress = gVar._proxyAddress;
			if (!proxyAddress.equals(gVar.NO_ADDRESS)) { 
				return proxyAddress;
			} else {
				throw new BindingException("Variable ["+varId+"] has no proxy address assigned");
			} // end if..else check for address
		} else {
			throw new BindingException("Variable ["+varId+"] unknown in global name space");
		} // end if..else check for known varid
	} // end getGlobalSpaceProxyAddress


	/**
		The method <i>getGlobalSpaceIdentifier</i> returns  an identifier 
		in the global name space, for the given reference. 
		
		@param  varId The identifier of the proxy 
		@return GlobalIdentifier	 The variable identifier in the global name space 
	*/ 
	
	public GlobalIdentifier getGlobalSpaceIdentifier (String varId) 
		throws BindingException
	{
		if (_globalNameSpace.containsKey(varId)) { 
			return (GlobalIdentifier) _globalNameSpace.get(varId);
		} else {
			throw new BindingException("Variable ["+varId+"] unknown in global name space");
		} // end if..else check for existence
	} // end getGlobalSpaceIdentifier


	/**
		The method <i>getGlobalSpaceIdentifierReference</i> returns a reference to an identifier 
		in the global name space, for the given proxy address. 
		
		@param proxyAddr The identifier of the proxy 
		@return String	 The variable identifier in the global name space referring to the given proxy, or NO_ID if unknown
	*/ 
	
	public String getGlobalSpaceIdentifierReference (String proxyAddr) 
	{  
		String result = this.NO_ID; 
		if (_proxiesToGlobalNameSpace.containsKey(proxyAddr)) { 
			result = (String) _proxiesToGlobalNameSpace.get(proxyAddr);
		} // end if.. check for known proxyAddr
		return result;
	} // end getGlobalSpaceIdentifier


	/**
		The method <i>getGlobalSpaceIdentifiers</i> returns an Iterator over identifiers
		in the global name space
		
		@return Iterator	 The iterator over variable identifiers in the global name space, referring to proxies
	*/ 
	
	public Iterator getGlobalSpaceIdentifiers () 
	{ 
		return _globalNameSpace.keySet().iterator();
	} // end getGlobalSpaceIdentifiers


	/** 
		The method <i>updateGlobalSpaceIdentifier</i> updates the given global 
		identifier in the global name space, and updates the table 
		mapping proxy address to global identifiers. 
	
		@param gId	The global identifier to be updated
	*/ 

	public void updateGlobalSpaceIdentifier (GlobalIdentifier gId) 
	{ 
		log("Updating global identifier ["+gId._varId+"]");
		_globalNameSpace.put(gId._varId, gId);
		_proxiesToGlobalNameSpace.put(gId._proxyAddress,gId._varId);
	}  // end updateGlobalSpaceIdentifier

	//-----------------------------------------------------------------
	// LOCAL NAME SPACE ACCESS
	//-----------------------------------------------------------------	
	
	/**
		The method <i>addLocalSpaceIdentifier</i> adds a Identifier to the
		given proxy in the given namespace
		
		@param varId	The variable identifier used as Identifier
		@param spaceId	The local namespace
		@param proxyAddr The proxy referred to (by address)
		@throws BindingException If the identifier cannot be added, or the local name space is unknown
		
	*/

	public void addLocalSpaceIdentifier (String varId, String spaceId, String proxyAddr)
		throws BindingException
	{ 
		log("Adding local space identifier with id ["+varId+"] to name space ["+spaceId+"]");	
		if (_localNameSpaces.containsKey(spaceId)) {  
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			LocalIdentifier locIdentifier = new LocalIdentifier (varId, proxyAddr, spaceId);
			locSpace.addIdentifier(locIdentifier); 
			_localNameSpaces.put(spaceId,locSpace);
		} else { 
			LocalNameSpace locSpace = new LocalNameSpace(spaceId);
			LocalIdentifier locIdentifier = new LocalIdentifier (varId, proxyAddr, spaceId);
			locSpace.addIdentifier(locIdentifier); 
			_localNameSpaces.put(spaceId,locSpace);
		} // end if..else check for known space 
	} // end addLocalSpaceIdentifier
 
	/**
		The method <i>addLocalSpaceProxyRelation</i> adds a proxy relation to the given namespace
		
		@param proxyRel	The proxy relation 
		@param spaceId	The local namespace
		@throws BindingException If the relation cannot be added, or the local name space is unknown
		
	*/

	public void addLocalSpaceProxyRelation (ProxyRelation proxyRel, String spaceId)
		throws BindingException
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			locSpace.addProxyRelation(proxyRel);
			_localNameSpaces.put(spaceId,locSpace);
		} else { 
			LocalNameSpace locSpace = new LocalNameSpace(spaceId);
			locSpace.addProxyRelation(proxyRel);
			_localNameSpaces.put(spaceId,locSpace);			
		} // end if..else check for known space
	} // end addLocalSpaceIdentifier




	/**
		The method <i>addLocalToGlobalIdentifier</i> links a given local identifier, in the 
		given local namespace, to the given identifier in the global namespace. Both the local 
		and the global identifiers are updated (and, also, the local name space of the former).
		
		@param	varId	The variable identifier in the given local namespace
		@param	spaceId	The local namespace
		@param	globalVarId	The identifier in the global namespace
		@throws BindingException If the Identifier cannot be added (usually, a link already exists)
	*/
	
	public void addLocalToGlobalIdentifier (String varId, String spaceId, String globalVarId)
		throws BindingException 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			LocalIdentifier locIdentifier = locSpace.getIdentifier(varId);
			if (_globalNameSpace.containsKey(globalVarId)) { 
				GlobalIdentifier globalIdentifier = (GlobalIdentifier) _globalNameSpace.get(globalVarId);
				globalIdentifier.addLocalIdentifier(locIdentifier);
				_globalNameSpace.put(globalVarId,globalIdentifier);
				locIdentifier._globalIdentifier = globalVarId;
				locSpace.updateIdentifier(locIdentifier);
				_localNameSpaces.put(spaceId,locSpace);
				if (!locIdentifier._proxyAddress.equals(locIdentifier.NO_ADDRESS)) { 
					_proxiesToGlobalNameSpace.put(locIdentifier._proxyAddress,globalVarId);
				} else { 	
					throw new BindingException("No proxy address known in local identifier ["+varId+"]");
				} // end if.. else check for known proxy address in local identifier
			} else { 
				throw new BindingException("Global identifier ["+globalVarId+"] unknown in global namespace");
			} // end if..else check for global id in global name space 
		} else { 
			throw new BindingException("Local name space ["+spaceId+"] unknown in monitor state");
		} // end if..else check for known space		
	} // end addLocalToGlobalIdentifier
		
	/** 
		The method <i>excludeIdentifier</i> puts the given variable identifier on a list of 
		identifiers which should not be considered for (further) processing. 
		
		@param varId	The variable identifier to be excluded
		@param spaceId	The local name space
	*/ 

	public void excludeIdentifier (String varId, String spaceId) 
	{ 
		log("Excluding identifier ["+varId+"] in local name space ["+spaceId+"]");
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			locSpace.addExcludedIdentifier(varId);
			_localNameSpaces.put(spaceId,locSpace);
		} else { 
			LocalNameSpace locSpace = new LocalNameSpace(spaceId);
			locSpace.addExcludedIdentifier(varId);
			_localNameSpaces.put(spaceId,locSpace);
		} // end if..else
	} // end excludeIdentifier


	/** 
		The method <i>excludeIdentifier</i> puts each of the given variable identifiers on the
		iterator onto a list of identifiers which should not be considered for (further) processing. 
		
		@param varId	The variable identifier to be excluded
		@param spaceId	The local name space
	*/ 

	public void excludeIdentifiers (Iterator varsIter, String spaceId) 
	{ 
		while (varsIter.hasNext()) { 
			String varId = (String) varsIter.next();
			excludeIdentifier(varId,spaceId);
		} // end while over varIds
	} // end excludeIdentifiers



	/** 
		The method <i>existsLocalSpace</i> returns a boolean to indicate whether 
		there is a local namespace with the given identifier
		
		@param spaceId	The identifier of the local namespace
		@return boolean Whether there is a local space with that identifier
	*/ 


	public boolean existsLocalSpace (String spaceId) {
		return _localNameSpaces.containsKey(spaceId);
	} // end existsLocalSpace


	/** 
		The method <i>existsLocalSpaceIdentifier</i> returns a boolean to indicate whether 
		there is a local identifier in the local namespace with the given identifier
		
		@param varId	The id of the local identifier
		@param spaceId	The identifier of the local namespace
		@return boolean Whether there is a local space with that identifier
	*/ 


	public boolean existsLocalSpaceIdentifier (String varId, String spaceId) 
		throws BindingException 
	{
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			return locSpace.existsIdentifier(varId);
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if.. check for local name space
	} // end existsLocalSpace



	/** 
		The method <i>existsLocalSpaceProxyAddress</i> returns a boolean to indicate whether 
		for the given variable identifier in the given local namespace, there is a known proxy
		address the variable refers to. 
		
		@param varId	The variable identifier in the local namespace
		@param spaceId	The identifier of the local namespace
		@return boolean Whether there is a known proxy address which the variable refers to
	*/ 

	public boolean existsLocalSpaceProxyAddress (String varId, String spaceId) { 
		boolean result = false; 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			try { 
				LocalIdentifier locIdentifier = locSpace.getIdentifier(varId);
				String proxyAddress = locIdentifier._proxyAddress;
				if (!proxyAddress.equals(locIdentifier.NO_ADDRESS)) { 
					result = true; 
				} // end if.. check for known address
			} catch (BindingException be) { 
				// do nothing -- if nothing, then nothing
			} // end try..catch
		} // end if.. check for local name space
		return result;
	} // end existsLocalSpaceProxyAddress

	/** 
		The method <i>existsLocalSpaceProxyRelation</i> returns a boolean indicating whether 
		in the given local name space, a proxy relation with the (template) information as
		provided by the given relation already exists. 
	*/ 

	public boolean existsLocalSpaceProxyRelation (String spaceId, ProxyRelation rel) 
		throws BindingException 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			return locSpace.existsProxyRelation(rel);
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if.. check for local name space
	} // end existsLocalSpaceProxyRelation



	/**
		The method <i>getLocalSpaceProxyAddress</i> returns a proxy address 
		for the given identifier in the given local name space. 
		
		@param  varId    The identifier in the local name space
		@param  spaceId	 The identifier of the local name space
		@return String	 The proxy referred to by the identifier in the global name space
		@throws BindingException If the local name space is unknown		
	*/ 
	
	public String getLocalSpaceProxyAddress (String varId, String spaceId) 
		throws BindingException 
	{  	
		String result = NO_ADDRESS; 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			LocalIdentifier locIdentifier = locSpace.getIdentifier(varId);
			result = locIdentifier._proxyAddress;
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if.. check for local name space
		return result;		
	} // end getLocalSpaceProxyAddress
	
	/** 
		The method <i>getLocalSpaceIdentifier</i> returns the LocalIdentifier object for 
		a given variable in a local name space, if it exists. If the variable is unknown 
		in the name space, or the name space itself is unknown, an exception is thrown. 
	
		@param varId	The variable
		@param spaceId	The local name space
		@return LocalIdentifier	The local identifier object for the given variable
		@throws BindingException If the identifier or the name space is unknown
	*/ 
	
	
	
	public LocalIdentifier getLocalSpaceIdentifier (String varId, String spaceId) 
		throws BindingException 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			return locSpace.getIdentifier(varId); 
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if.. check for local name space		
	} // end getLocalSpaceIdentifier
	
	/**
		The method <i>getLocalSpaceIdentifier</i> returns a reference to an identifier 
		in the given local name space, for the given proxy address. Returns NO_ID if not known 
		
		@param spaceId	The identifier of the local name space
		@param proxyAddr The identifier of the proxy 
		@return String	 The variable identifier in the local name space referring to the given proxy
		@throws BindingException If the local name space is unknown		
	*/ 
	
	public String getLocalSpaceIdentifierReference (String spaceId, String proxyAddr) 
		throws BindingException 
	{ 
		String result = NO_ID; 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			Iterator idIter = locSpace.getIdentifiers(); 
			while (idIter.hasNext()) { 
				String locId = (String) idIter.next();
				LocalIdentifier locIdentifier = (LocalIdentifier) locSpace.getIdentifier(locId); 
				if (locIdentifier != null) { 
					if (locIdentifier._proxyAddress.equals(proxyAddr)) {
						result = locIdentifier._varId;
					} // end if.. check for matching proxy address 
				} // end if.. check for non-null
			} // end while
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if.. check for local name space		
		return result;
	} // end getLocalSpaceIdentifier 	
	
	/**
		The method <i>getLocalSpaceIdentifiers</i> returns an Iterator over identifiers 
		in the given local name space. If the space is not known, or there are no identifiers, 
		a non-null but empty iterator is returned. 
		
		@param spaceId	The identifier of the local name space
		@return Iterator The iterator over variable identifiers in the local name space
	*/ 
	
	public Iterator getLocalSpaceIdentifiers (String spaceId) 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace localSpace = (LocalNameSpace) _localNameSpaces.get(spaceId); 
			return localSpace.getIdentifiers();
		} else { 	
			Vector result = new Vector();
			return result.iterator();
		} // end if..else check for local space
	} // end getLocalSpaceIdentifiers


	/**
		The method <i>getLocalSpaceProxyRelation</i> returns proxy relation for the given label, 
		if it exists. 
				
		@param  relLabel The relation label
		@param  spaceId	The identifier of the local name space
		@return Iterator The iterator over variable identifiers in the local name space
	*/ 
	
	public ProxyRelation getLocalSpaceProxyRelation (String relLabel, String spaceId) 
		throws BindingException 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace localSpace = (LocalNameSpace) _localNameSpaces.get(spaceId); 
			return localSpace.getProxyRelation(relLabel);
		} else { 	
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if..else check for local space
	} // end getLocalSpaceProxyRelation



	/**
		The method <i>getLocalSpaceProxyRelations</i> returns an Iterator over identifiers 
		for proxy relations in the given local name space in which the given variable
		identifier is involved
		
		@param  varId	The variable identifier
		@param  spaceId	The identifier of the local name space
		@return Iterator The iterator over variable identifiers in the local name space
	*/ 
	
	public Iterator getLocalSpaceProxyRelations (String varId, String spaceId) 
		throws BindingException 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace localSpace = (LocalNameSpace) _localNameSpaces.get(spaceId); 
			return localSpace.getProxyRelations(varId);
		} else { 	
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if..else check for local space
	} // end getLocalSpaceProxyRelations


	public int getLocalSpaceProxyRelationsSize (String spaceId) 
		throws BindingException
	{
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace localSpace = (LocalNameSpace) _localNameSpaces.get(spaceId); 
			return localSpace.getProxyRelationsSize();
		} else { 	
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if..else check for local space		

	} // end getLocalSpaceProxyRelationsSize


	/** 
		The method <i>isExcluded</i> returns a boolean to indicate whether 
		for the given variable identifier in the given local namespace, it is known 
		that the variable has been excluded from further processing
						
		@param varId	The variable identifier in the local namespace
		@param spaceId	The identifier of the local namespace
		@return boolean Whether the given identifier has been excluded from further processing
	*/ 

	public boolean isExcluded (String varId, String spaceId) 
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			return locSpace.isExcluded(varId);
		} else {
			return false;
		} // end if..else check for namespace
	} // end isExcluded

	/**
		The method <i>removeLocalSpaceIdentifier</i> removes the variable identifier from 
		the given local name space. This includes removing links to proxy addresses, and 
		identifiers in the global name space. 

		@param varId	The variable identifier to be removed
		@param spaceId	The local name space
		@throws BindingException If the local name space is unknown		
	*/ 

	public void removeLocalSpaceIdentifier (String varId, String spaceId) 
		throws BindingException
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			locSpace.removeIdentifier(varId);
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if..else check for namespace		
	} // end removeLocalSpaceIdentifier


	/**
		The method <i>removeLocalSpaceProxyRelation</i> removes the proxy relation from 
		the given local name space. 

		@param relLabel	The proxy relation to be removed
		@param spaceId	The local name space
		@throws BindingException If the local name space is unknown		
	*/ 

	public void removeLocalSpaceProxyRelation (String relLabel, String spaceId) 
		throws BindingException
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);
			locSpace.removeProxyRelation(relLabel);
		} else {
			throw new BindingException("Local name space ["+spaceId+"] unknown in the monitor state"); 
		} // end if..else check for namespace		
	} // end removeLocalSpaceProxyRelation


	/**
		The method <i>updateLocalSpaceIdentifier</i> updates an existing Identifier in the given namespace
		
		@param varId	The variable identifier used as Identifier
		@param spaceId	The local namespace
		@param proxyAddr The proxy referred to (by address)
		@throws BindingException If the identifier cannot be added, or the local name space is unknown
		
	*/

	public void updateLocalSpaceIdentifier (String varId, String spaceId, String proxyAddr)
		throws BindingException
	{ 
		log("Updating local identifier ["+varId+"] in local name space ["+spaceId+"]");
		if (_localNameSpaces.containsKey(spaceId)) {  
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId); 
			LocalIdentifier locIdentifier = new LocalIdentifier (varId, proxyAddr, spaceId);
			locSpace.updateIdentifier(locIdentifier);
			_localNameSpaces.put(spaceId,locSpace);
		} else { 
			LocalNameSpace locSpace = new LocalNameSpace (spaceId);
			LocalIdentifier locIdentifier = new LocalIdentifier (varId, proxyAddr, spaceId);
			locSpace.updateIdentifier(locIdentifier);
			_localNameSpaces.put(spaceId,locSpace);
		} // end if..else check for known space
	} // end updateLocalSpaceIdentifier  

	/**
		The method <i>updateLocalSpaceIdentifier</i> updates an existing Identifier in the given namespace
		
		@param locId	The identifier
		@throws BindingException If the identifier cannot be added, or the local name space is unknown
		
	*/

	public void updateLocalSpaceIdentifier (LocalIdentifier locId, String spaceId)
		throws BindingException
	{ 
		if (_localNameSpaces.containsKey(spaceId)) { 
			LocalNameSpace locSpace = (LocalNameSpace) _localNameSpaces.get(spaceId);			
			locSpace.updateIdentifier(locId);
			_localNameSpaces.put(spaceId,locSpace);
		} else { 
			LocalNameSpace locSpace = new LocalNameSpace(spaceId);			
			locSpace.updateIdentifier(locId);
			_localNameSpaces.put(spaceId,locSpace);
		} // end if..else check for known space
	} // end updateLocalSpaceIdentifier


	public void log (String m) { 
		if (logging) { System.out.println("[AbstractMonitorState] "+m); }
	}

} // end class
