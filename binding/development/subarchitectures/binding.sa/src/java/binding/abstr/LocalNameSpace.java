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
import java.util.TreeSet;

//=================================================================
// JAVADOC CLASS DOCUMENTATION
//=================================================================

/** 
	The class <b>LocalNameSpace</b> defines a data structure for 
	keeping track of the links between local variable identifiers, 
	and proxy addresses. 

	@started 071107
	@version 071106
	@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public class LocalNameSpace {

	//=================================================================
	// GLOBAL DATA STRUCTURES
	//=================================================================

	/** TreeSet of identifiers (String) that should be excluded from further processing */ 
	TreeSet<String> _localNameSpaceExcludes; 

	/** Hashtable with the local identifiers */
	Hashtable<String,LocalIdentifier> _varNameSpace; 

	/** Proxy addresses to local identifiers */ 
	Hashtable<String,String> _proxyAddressesToIdentifiers; 

	/** Proxy relations, stored by label fromId+relMode+toId (key) and ProxyRelation (value) */ 
	Hashtable<String,ProxyRelation> _proxyRelations;


	/** QUICK INDEX: from identifier to proxy relations, per identifier (Key for _varNameSpace) 
		a list of proxy relation labels is stored (Key for _proxyRelations) */
	Hashtable<String,TreeSet>_QIDX_idToProxyRelations;

	String _spaceId;
	
	
	public boolean logging = false;

	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================
		
	public LocalNameSpace () { 
		init();
	} // end constructor

	public LocalNameSpace (String id) { 
		init();
		_spaceId = id; 
	} // end constructor

	protected void init() { 
		_varNameSpace = new Hashtable<String,LocalIdentifier>();
		_spaceId = "";
		_localNameSpaceExcludes = new TreeSet<String>();
		_proxyAddressesToIdentifiers = new Hashtable<String,String>();
		_proxyRelations = new Hashtable<String,ProxyRelation>();
		_QIDX_idToProxyRelations = new Hashtable<String,TreeSet>();
	} // end init

	//=================================================================
	// ACCESSOR METHODS
	//=================================================================

	/** 
		The method <i>addExcludedIdentifier</i> adds the given variable
		identifier to the list of identifiers which are to be excluded
		from further processing
	
		@param lvar	The identifier to be exluded
	*/ 

	public void addExcludedIdentifier (String varId) { 
		if (!_localNameSpaceExcludes.contains(varId)) { 
			_localNameSpaceExcludes.add(varId);
		} // end if.. check whether var already present; only add once
	} // end addExcludes

	/** 
		The method <i>addIdentifier</i> adds the given identifier to the
		local name space. 
		
		@param var	The identifier object
		@throws BindingException Thrown if an identifier with the given name already exists
	*/ 

	public void addIdentifier (LocalIdentifier lvar)
		throws BindingException 
	{
		if (!_varNameSpace.containsKey(lvar._varId)) { 
			_varNameSpace.put(lvar._varId,lvar);
			_proxyAddressesToIdentifiers.put(lvar._proxyAddress,lvar._varId);
		} else { 
			throw new BindingException("[ERROR:LocalNameSpace] Local variable identifier ["+lvar._varId+"] already exists in this name space");
		} // end if..else check for existence
	} // end addIdentifier

	/** 
		The method <i>addProxyRelation</i> stores the given proxy relation, keyed by 
		a label composed from fromId+relMode+toId (with the id's the local identifier
		variable names).
		
		@param rel	The proxy relation to be added
	*/ 

	public void addProxyRelation (ProxyRelation rel) { 
		String fromId = _proxyAddressesToIdentifiers.get(rel._fromProxyAddress);
		String toId   = _proxyAddressesToIdentifiers.get(rel._toProxyAddress);
		String label  = fromId+"+"+rel._proxyMode+"+"+toId;
		
		log("Adding a proxy relation ["+label+"] from ["+fromId+"] to ["+toId+"]");
		
		_proxyRelations.put(label,rel);
		// update the quick-index table, for both from- and to-identifiers
		TreeSet proxyRels = null;
		proxyRels = (TreeSet) _QIDX_idToProxyRelations.get(fromId);
		if (proxyRels != null) { 
			proxyRels = (TreeSet) _QIDX_idToProxyRelations.get(fromId);
		} else { 	
			proxyRels = new TreeSet();
		} // end if.. else check
		proxyRels.add(label);
		_QIDX_idToProxyRelations.put(fromId,proxyRels);

		proxyRels = (TreeSet) _QIDX_idToProxyRelations.get(toId);
		if (proxyRels != null) { 
			proxyRels = (TreeSet) _QIDX_idToProxyRelations.get(toId);
		} else { 
			proxyRels = new TreeSet();
		} // end if.. else check 
		proxyRels.add(label);
		_QIDX_idToProxyRelations.put(toId,proxyRels);		
	} // end addProxyRelation


	/** 
		The method <i>existsIdentifier</i> returns a boolean indicating whether
		a variable with the given name exists in the local name space
	*/ 

	public boolean existsIdentifier (String varId) {
		return _varNameSpace.containsKey(varId);
	} // end existsIdentifier


	/** 
		The method <i>existsProxyRelation</i> returns a boolean indicating whether
		a proxy relation with the given parameters exists in the local name space
	*/ 

	public boolean existsProxyRelation(ProxyRelation rel) { 
		String fromId = _proxyAddressesToIdentifiers.get(rel._fromProxyAddress);
		String toId   = _proxyAddressesToIdentifiers.get(rel._toProxyAddress);
		String label  = fromId+"+"+rel._proxyMode+"+"+toId;
		return _proxyRelations.containsKey(label);
	} // end existsProxyRelation


	/** 
		The method <i>getIdentifiers</i> returns an iterator over the
		local identifiers (String: name of the identifier). The individual 
		identifiers can be retrieved 

	*/ 
	
	public Iterator getIdentifiers () { 
		return _varNameSpace.keySet().iterator();
	} // end getIdentifiers
	
	/**
		The method <i>getIdentifier</i> returns the LocalIdentifier object
		with information about the given variable identifier, if it is 
		known in the local name space. If not, an exception is thrown. 
		
		@param varId	The variable identifier
		@throws	BindingException Thrown if the variable identifier is unknown. 
	*/ 
	
	public LocalIdentifier getIdentifier (String varId)
		throws BindingException 
	{ 
		log("Known local identifiers: "+_varNameSpace.keySet());
		if (_varNameSpace.containsKey(varId)) { 
			return (LocalIdentifier) _varNameSpace.get(varId);
		} else {
			throw new BindingException("[ERROR:LocalNameSpace] Local variable identifier ["+varId+"] unknown in name space ["+_spaceId+"]");
		} // end if..else checking for existence of the variable
	} // end getIdentifier


	/** 
		The method <i>getProxyRelation</i> returns the ProxyRelation object with the 
		given label. If there is no such object for the label, and exception is thrown.
		
		@param	relLabel	The label for the object
		@throws	BindingException If there is no object for the given label
		@return ProxyRelation	The relation object for the label
	*/ 

	public ProxyRelation getProxyRelation (String relLabel) 
		throws BindingException 
	{ 
		if (_proxyRelations.containsKey(relLabel)) { 
			return (ProxyRelation) _proxyRelations.get(relLabel);
		} else {
			throw new BindingException("[ERROR:LocalNameSpace] Proxy relation with label ["+relLabel+"] unknown in this name space");
		} // end if..else check for availability
	} // end getProxyRelation

	/** 
		The method <i>getProxyRelations</i> returns an iterator over labels for relations
		in which the given identifier is involved. (A label can be used to retrieve a
		relation, or remove a relation.)
		
		@param varId		The identifier
		@return Iterator	An iterator over relation labels (String)
		@throws BindingException If there are no proxy relations for the variable
	*/ 

	public Iterator<String> getProxyRelations (String varId) 
	{
		if (_QIDX_idToProxyRelations.containsKey(varId)) { 
			TreeSet<String> relLabels = (TreeSet<String>) _QIDX_idToProxyRelations.get(varId);
			return relLabels.iterator();
		} else { 
			// throw new BindingException("[ERROR:LocalNameSpace] For variable ["+varId+"] there are no proxy relations in this name space");
			TreeSet result = new TreeSet();	
			return result.iterator();
		} // end if.. check for available relations
	} // end getProxyRelations

	/** 
		The method <i>getProxyRelationsSize</i> returns the size of the table maintaining the proxy relations. 
	*/ 
	
	public int getProxyRelationsSize () { 
		return _proxyRelations.size();
	} // end getProxyRelationsSize



	/** 
		The method <i>isExcluded</i> returns a boolean indicating whether the 
		given variable identifier is known to be excluded from further processing. 
		
		@param varId	The variable identifier to check for
		@return boolean Whether the given identifier is known to be excluded from further processing
	*/ 

	public boolean isExcluded (String varId) { 
		log("Known local excludes: "+_localNameSpaceExcludes);
		return _localNameSpaceExcludes.contains(varId);
	} // end isExcluded


	/** 
		The method <i>removeIdentifier</i> removes an identifier with the 
		given name, if it exists. 

		@param varId	The variable identifier to be removed
	*/ 
	
	public void removeIdentifier (String varId) {
		if (_varNameSpace.containsKey(varId)) { 
			_varNameSpace.remove(varId);
			_QIDX_idToProxyRelations.remove(varId);
		} // end if..check for existence
	} // end removeIdentifier
	
	
	/** 
		The method <i>removeProxyRelation</i> removes a relation with the 
		given label, if it exists. 

		@param relLabel	The label of the relation to be removed
	*/ 
	
	public void removeProxyRelation (String relLabel) {
		if (_proxyRelations.containsKey(relLabel)) { 
			ProxyRelation proxyRel = (ProxyRelation) _proxyRelations.get(relLabel);
			_proxyRelations.remove(relLabel);
			TreeSet proxyRels = null;
			// remove the relation from the FROM identifier's list
			String varId = _proxyAddressesToIdentifiers.get(proxyRel._fromProxyAddress); 
			if (!varId.equals(proxyRel.NO_ADDRESS)) {  
				proxyRels = (TreeSet) _QIDX_idToProxyRelations.get(varId);
				if (proxyRels != null) { 
					proxyRels.remove(relLabel);
					_QIDX_idToProxyRelations.put(varId,proxyRels);
				} // end if.. check there are relations
			} // end if.. check there is a from-address
			// remove the relation from the TO identifiers list
			varId = _proxyAddressesToIdentifiers.get(proxyRel._toProxyAddress); 
			if (!varId.equals(proxyRel.NO_ADDRESS)) {
				proxyRels = (TreeSet) _QIDX_idToProxyRelations.get(varId);
				if (proxyRels != null) { 
					proxyRels.remove(relLabel);
					_QIDX_idToProxyRelations.put(varId,proxyRels);
				} // check there are relations
			} // check there is a to-address
		} // end if..check for existence
	} // end removeProxyRelation 
	
	/** 
		The method <i>updateIdentifier</i> checks for there being a variable identifier of the given id in 
		the name space, and if so, updates the object. 	*/
	
	public void updateIdentifier (LocalIdentifier lvar) 
	{ 
		_varNameSpace.put(lvar._varId,lvar);
		_proxyAddressesToIdentifiers.put(lvar._proxyAddress,lvar._varId);			
	} // end updateIdentifier
	
	public void log (String m) { 
		if (logging) { System.out.println("[LocalNameSpace] "+m); }
	}
	
	
	
} // end class LocalNameSpace
