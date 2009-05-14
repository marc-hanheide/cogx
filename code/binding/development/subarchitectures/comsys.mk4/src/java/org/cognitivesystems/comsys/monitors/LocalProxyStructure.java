//=================================================================
// Copyright (C) 2008 Geert-Jan M. Kruijff (gj@acm.org)
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

package org.cognitivesystems.comsys.monitors;

//=================================================================
// IMPORTS
//=================================================================

import java.util.Iterator;
import java.util.TreeMap;
import java.util.TreeSet;


//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 The class <b>LocalProxyStructure</b> implements a representation
 for storing proxy information locally in a subarchitecture-specific
 binding monitor. The purpose of this representation is to separate
 the creation of a proxy from the actual storing of that proxy on 
 the binding SA working memory. This separation enables e.g. checking 
 whether the created proxy structure is equal to a proxy already 
 present on the binding SA working memory -- thus avoiding spurious
 updates. 
 <p>
 The class implements methods for setting features and relations for
 the proxy; checking equality of the structure against another local
 proxy structure; initializing the structure using a local proxy 
 structure; and cycling over the content (features, relations) of 
 the local structure. 
 <p>
 To identify the local proxy structure, methods are provided for 
 setting and getting a global namespace identifier. 
 
 @started 080616
 @version 080619
 @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class LocalProxyStructure {

    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	// ALL THE VARIABLES ARE INITIALIZED IN THE "INIT()" METHOD

	// Boolean whether to provide any logging output
    boolean logging; 

	// Hashmap storing the features for the structure
	TreeMap proxyFeatures; 
	
	// Hashmap storing the relations for the structure
	TreeMap proxyRelations; 

	// Global namespace identifier for the proxy structure
	String gid;
	
	// Set for storing which graph node identifiers are covered by this proxy
	TreeSet coveredNodes; 


	// Set for storing content status labels
	TreeSet contentStatus;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
	The basic constructor initializes the internal variables
     */

    public LocalProxyStructure () {
		init();
    } // end constructor


	/**
	The unary constructor initializes the internal variables using 
	the values in the provided proxy structure. Feature/values and relations are copied. 
	
	@param LocalProxyStructure initStructure	The structure to be used to initialize the object
	*/ 

	public LocalProxyStructure (LocalProxyStructure initStructure) 
		throws NullPointerException 
	{ 
		init();
		try { 
		// cycle over the init features, set the features
			for (Iterator<String> featsIter = initStructure.getFeatures(); featsIter.hasNext(); ) { 
				String feature = featsIter.next();
				String value   = initStructure.getFeatureValue(feature);
				this.setFeature(feature,value);
			} // end for
			// cycle over the init relations, set the relations
			for (Iterator relsIter = initStructure.getRelations(); relsIter.hasNext(); ) { 
				PendingProxyRelation relation = (PendingProxyRelation) relsIter.next();
				String relationKey = relation.headNomVar+"+"+relation.depNomVar+"+"+relation.relMode+"+"+relation.tempFrameType;
				proxyRelations.put(relationKey, relation);
			} // end for
		} catch (NullPointerException npe) {  
			throw new NullPointerException (npe.getMessage());
		} // end try..catch
	} // end constructor


	/**
	Initializes the internal variables
	*/
	
	protected void init() { 
		logging = false;
		proxyFeatures = new TreeMap();
		proxyRelations = new TreeMap();
		gid = null;
		coveredNodes = new TreeSet();
		contentStatus = new TreeSet();
	} // end init

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** 
	Add a label to reflect what type of content this proxy structure provides. Status
	drives the decisions made in the binding monitor, where to store the proxies for 
	this structure. 	
	*/ 

	public void addContentStatus (String status) { 
		contentStatus.add(status);
	} // end addContentStatus


	public void addCoveredNode (String nodeId) { 
		if (!coveredNodes.contains(nodeId)) { 
			coveredNodes.add(nodeId);
		} // end if.. 
	} // end 

	
	public void addCoveredNodes (TreeSet nodeIds) { 
		for (Iterator<String> nodesIter = nodeIds.iterator(); nodesIter.hasNext(); ) { 
			addCoveredNode(nodesIter.next()); 
		} // end for
	} // end addCoveredNodes


	public void addRelations (Iterator<PendingProxyRelation> relsIter) { 
		while (relsIter.hasNext()) { 
			addRelation((PendingProxyRelation)relsIter.next());
		} 
	} 


	/** */

	public boolean addRelation (PendingProxyRelation ppr) { 
		return this.addRelation(ppr.headNomVar,ppr.depNomVar,ppr.relMode,ppr.tempFrameType);
	} 

	/**
	The method <i>addRelation</i> adds a relation of a given mode between 
	two nominals, identified by their nominal variables. The status of
	this relation is assumed to be "ASSERTED." The method returns a boolean
	indicating whether the relation could be added -- no duplicates are 
	allowed. 
	
	@param head			The head of the relation (outgoing node)
	@param dependent	The dependent of the relation (incoming node)
	@param mode			The mode of the relation
	@return boolean		Whether the relation could be added. 
	*/
	
	public boolean addRelation (String head, String dependent, String mode) { 
			return this.addRelation(head,dependent,mode,"ASSERTED");
	} // end addRelation 
	
	/**
	The method <i>addRelation</i> adds a relation of a given mode between 
	two nominals, identified by their nominal variables, and a given
	status. 
	
	@param head			The head of the relation (outgoing node)
	@param dependent	The dependent of the relation (incoming node)
	@param mode			The mode of the relation
	@param status		The temporal / epistemological status of the relation
	*/
	
	public boolean addRelation (String head, String dependent, String mode, String status) { 
		boolean result = false; 
		String relationKey = head+"+"+dependent+"+"+mode+"+"+status;
		if (!proxyRelations.containsKey(relationKey)) { 
			PendingProxyRelation relation = new PendingProxyRelation (head, dependent, mode, status);
			proxyRelations.put(relationKey,relation);
			result = true;
		} // end if.. check for relation already present
		return result;
	} // end addRelation 

	/**
	The method <i>concatFeature</i> adds the provided value to the 
	given feature's already present values. Adding is done by concatenating
	the value as String to the present values, separated by a space. 
	
	@param feature	The feature to be set
	@param value	The value to be used
	@throws NullPointerException Whether the value could be concatenated
	*/ 

	public boolean concatFeature (String feature, String value) { 
		boolean result = false; 
		if (isSetFeature(feature)) { 
			try { 
				String oldValue = this.getFeatureValue(feature);
				String newValue = oldValue + " " + value;
				this.updateFeature(feature,newValue);
			} catch (NullPointerException npe) { 
				throw new NullPointerException (npe.getMessage());
			} // end try..catch
		} else { 
			this.setFeature(feature,value);
			result = true;
		} // end if..else check for presence feature
		return result;
	} // 

	/**
	The method <i>fuse</i> adds the features and relations from the provided 
	proxy structure to the current one. Fusion is conservative: it does not 
	overwrite any already existing features or relations. 

	@param fusePRX	The proxy structure to be fused with
	*/ 

	public void fuse (LocalProxyStructure fusePRX) {
		try { 
			for (Iterator<String> featsIter = fusePRX.getFeatures(); featsIter.hasNext(); ) { 
				String feature = featsIter.next();
				if (!this.isSetFeature(feature)) { this.setFeature(feature,fusePRX.getFeatureValue(feature)); }
			} // end for over features
			for (Iterator<PendingProxyRelation> relsIter = fusePRX.getRelations(); relsIter.hasNext(); ) {
				PendingProxyRelation ppr = relsIter.next(); 
				this.addRelation(ppr);
			} // end for over relations
		} catch (NullPointerException npe) { 
			// not going to happen as we are iterating over existing content
		}
	} // end fuse


	public Iterator<String> getContentStatus() { 
		return contentStatus.iterator();
	} // end getContentStatus


	public Iterator<String> getCoveredNodes () { 
		return coveredNodes.iterator();
	} // end getCoveredNodes

	/** 
	The method <i>getFeatures</i> returns an iterator over the features in 
	the local proxy structure

	@return Iterator	The iterator over the features (String) 
	*/ 

	public Iterator getFeatures () { 
		return proxyFeatures.keySet().iterator();
	} // end getFeatures

	/** 
	The method <i>getFeatureValue</i> returns the value for the given feature, 
	if that feature is present. If there is no such feature, an exception is 
	thrown. 
	
	@param feature	The feature for which a value should be returned
	@return String	The value of the feature (if present)
	@throws NullPointerException If the feature is not present 
	*/ 

	public String getFeatureValue (String feature) 
		throws NullPointerException
	{ 
		if (isSetFeature(feature)) { 
			return (String) proxyFeatures.get(feature);
		} else { 
			throw new NullPointerException("Feature ["+feature+"] is null in local proxy structure");
		} // end if..else check for presence of feature
	} // end getFeatureValue


	/**
	The method <i>getGlobalID</i> returns the global namespace identifier for this (locally created) proxy structure. 
	This method makes it possible to identify the created proxy structure within the monitor state namespaces

	@return String	The global namespace identifier for this local proxy structure
	*/ 

	public String getGlobalID () { return gid; } 


	/**
	The method <i>getRelations</i> returns an iterator over the relations in 
	the local proxy structure. 
	
	@return Iterator	The iterator over the relations (PendingProxyRelation)
	*/ 

	public Iterator getRelations () { 
		return proxyRelations.values().iterator();
	} // end getRelations


	/** 
	The method <i>hasContentStatus</i> returns a boolean indicating whether the proxy structure 
	has the given content status. 
	
	@return boolean		Whether the proxy structure has been assigned the given content status
	*/ 

	public boolean hasContentStatus (String status) { 
		return contentStatus.contains(status);
	} // end hasContentStatus



	public boolean isSetContentStatus () { 
		return (contentStatus.size() > 0); 
	} 


	/**
	The method <i>hasFeatureValue</i> returns a boolean indicating whether the
	local proxy structure has the given feature-value pair. 

	@return boolean		Whether the given feature-value pair exists 
	*/ 

	public boolean hasFeatureValue (String feature, String value) { 
		boolean result = false; 
		if (isSetFeature(feature)) { 
			result = value.equals((String)proxyFeatures.get(feature)); 
		} // end if.. check whether feature has been set at all, then check equality
		return result;
	} // end hasFeatureValue


	/**
	The method <i>hasRelation</i> returns a boolean indicating whether the
	local proxy structure has the given relation. 

	@return boolean		Whether the given relation exists 
	*/ 

	public boolean hasRelation(PendingProxyRelation relation) { 
		String relationKey = relation.headNomVar+"+"+relation.depNomVar+"+"+relation.relMode+"+"+relation.tempFrameType;
		return proxyRelations.containsKey(relationKey);
	} // end hasRelation

	/**
	The method <i>isSetFeature</i> checks whether the given feature
	has already been set for the current proxy structure
	
	@return boolean Indicates whether the feature exists		
	*/ 

	public boolean isSetFeature (String feature) { 
		return proxyFeatures.containsKey(feature);
	} // end existsFeature


    /**
     The method <i>setFeature</i> sets the given feature to the given
	 value. This method returns true if the feature could be set, false
	 if not -- a feature can be set, if it does not yet exist. 
	 
	 @param feature	The feature to be set
	 @param value	The value to be used
	 @return boolean Indicating whether the feature could be set 
     */

	public boolean setFeature (String feature, String value) { 
		boolean result = false; 
		if (!isSetFeature(feature)) { 
			proxyFeatures.put(feature,value);
			result = true;
		} // end if.. check whether feature can be set
		return result;
	} // end setFeature

	/** 
	The method <i>setGlobalID</i> sets the global namespace identifier for this local proxy structure. 
	
	@param	id	The identifier to be used
	*/ 
	
	public void setGlobalID (String id) { gid = id; }

	/** 
	The method <i>updateFeature</i> updates (replaces) the value of 
	the given feature with the provided value. 
	
	@param feature	The feature to be set
	@param value	The value to be used
	*/
	
	public void updateFeature (String feature, String value) { 
		proxyFeatures.put(feature,value);
	} // end updateFeature

    //=================================================================
    // I/O METHODS
    //=================================================================

	/**
		The method <i>equals</i> returns a short indicating equality 
		between the current local proxy structure and another structure
		to compare to. The possible short values: 
		<ul> 
		<li> "0" indicates inequality
		<li> "1" indicates equal features, but no equal relations
		<li> "2" indicates equal features and equal relations
		</ul>

		@param comp		The structure to be compared with
		@return short	Short-flag indicating equality status
	*/ 

	public short equals (LocalProxyStructure comp) { 
		boolean continueChecking = true; 
		short result = 0;
		try { 
		// cycle over the init features, set the features
		// result is set to 1, getting reset to 0 as soon as inequal feature/value is found
			if (this.numberOfFeatures() == comp.numberOfFeatures()) { 
				result = 1;
				for (Iterator featsIter = comp.getFeatures(); continueChecking && featsIter.hasNext(); ) { 
					String feature = (String) featsIter.next();
					String value   = comp.getFeatureValue(feature);
					if (!this.hasFeatureValue(feature,value)) {
						continueChecking = false; 
						result = 0;
					} // end if.. check for presence of matching feature/value
				} // end for
			} else { 
				continueChecking = false;
			} // end if..else only compare if equal number of features
			// cycle over the init relations, set the relations
			// result is set to 2, getting reset to 0 as soon as inequal relation is found
			if (continueChecking) { 
				if (this.numberOfRelations() == comp.numberOfRelations()) { 
					result = 2;
					for (Iterator relsIter = comp.getRelations(); continueChecking && relsIter.hasNext() ; ) { 
						PendingProxyRelation relation = (PendingProxyRelation) relsIter.next();
						if (!this.hasRelation(relation)) { 
							continueChecking = false;
							result = 0;		
						} // end if.. check for presence of matching relation
					} // end for
				} else {
					result = 0;
					continueChecking = false;
				} // end if..else only check if equal number of relations
			} // end if.. check whether to continue checking
		} catch (NullPointerException npe) {  
			result = 0;
		} // end try..catch
		return result;
	} // end equals


	/**
	The method <i>numberOfFeatures</i> returns the number of features this structure has. 
	
	@return int		The number of features in the feature map
	*/ 

	public int numberOfFeatures () { 
		return proxyFeatures.size();
	} // end numberOfFeatures
	
	/**
	The method <i>numberOfRelations</i> returns the number of relations this structure has. 
	
	@return int		The number of relations in the relation map
	*/ 

	public int numberOfRelations () { 
		return proxyRelations.size();
	} // end numberOfRelations	

    //=================================================================
    // MISCELLANEOUS METHODS
    //=================================================================

    private void log (String m) {
        if (logging) { System.out.println("[LocalProxyStructure] "+m); }
    }
	
	
	public String toString () { 
		String result = "Local proxy structure ["+gid+"]\n";
		for (Iterator<String> featsIter = this.getFeatures(); featsIter.hasNext(); ) { 
			String feature = featsIter.next();
			String value   = (String)proxyFeatures.get(feature);
			result = result + "["+feature+"] = ["+value+"]\n";
		} 
		return result;
	} // end toString
	

} // end class definition 


