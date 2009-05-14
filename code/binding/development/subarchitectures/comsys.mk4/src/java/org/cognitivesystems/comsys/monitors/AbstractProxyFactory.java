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

package org.cognitivesystems.comsys.monitors;

//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;
import org.cognitivesystems.comsys.general.CacheWrapper;
import org.cognitivesystems.comsys.monitors.ComSysBindingMonitor;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.Vector;

// ----------------------------------------------------------------
// LF imports
// ----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFEssentials.*;
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;


//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
The class <b>AbstractProxyFactory</b> implements the basic behaviors
for producing a proxy, or a collection of proxies and proxy relations. 
The proxies are written to the working memory of the binder, 
through the monitor associated with the  factory. 

@version 071002 (started 071002)
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public abstract class AbstractProxyFactory 
	implements ProxyFactory
	
{

	//=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	/** The sort of the packed nominal on which the factory works*/ 
	protected String rootSort;

	/** Flag whether proxies should be checked for hypothetical nature; by default this is true */ 
	boolean hypoCheck; 
	
	
	/** Local proxy structure which the proxy factory generates */ 
	protected LocalProxyStructure proxyStructure; 
	
	/** Relations which have been added */ 
	protected Vector<PendingProxyRelation> pendingRels; 

	
	/** Fast access to the cache with discourse referents */ 
	protected CacheWrapper _discRefsAccess; 

	/** Logging flag */
	protected boolean logging;

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public AbstractProxyFactory () { 
		init();
	} // end constructor
	
	private void init () { 
		rootSort = "abstract";
		hypoCheck = true;
		_discRefsAccess = null;
		logging = true; 
		proxyStructure = new LocalProxyStructure();
		pendingRels = new Vector<PendingProxyRelation>();

	} // end init
	
    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

	/** Returns the ontological sort of the root nominal from which the factory produces its proxies. */ 
	public String getRootSort () { return rootSort; } 

	/** Sets the cache with discourse referents to be used. */ 
	public void setDiscRefs (CacheWrapper dr) { 
		_discRefsAccess = dr;
	} 

	/** Sets the flag for checking hypothetical nature of proxies */
	public void setHypotheticalCheck (boolean c) { hypoCheck = c; }

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

	/**
		The method <i>addBasicContent</i> adds to the <tt>proxyStructure</tt> the discourse referent, proposition, and concept for the given nominal. 
	*/

	protected void addBasicContent (PackedNominal nom) { 
		// Add the nominal variable as a feature
		proxyStructure.setFeature("DiscRef",_discRefsAccess.getDiscRef(nom.nomVar));
		log("Discref: "+ _discRefsAccess.getDiscRef(nom.nomVar)+" for ["+nom.nomVar+"]");
		// Add the proposition to the proxy 
		proxyStructure.setFeature("Proposition",nom.prop.prop);
		log("Proposition: ["+nom.prop.prop+"]");
		// use the proposition as the a-modal concept
		proxyStructure.setFeature("Concept",nom.prop.prop);
		// Add the features to the nominal
		ArrayList<PackedFeature> packedFeats = new ArrayList<PackedFeature>(Arrays.asList(nom.feats));
		Iterator featsIter = packedFeats.iterator();
		addFeatures(featsIter);
	} // end addBasicContent

	/** 
		The method <i>addFeatures</i> takes an iterator over the features for a packed nominal, 
		and adds these features to the current proxy. 
		
		@param featsIter The iterator over (PackedFeature) features  
	*/ 
	
	protected void addFeatures (Iterator featsIter) 
	{ 
		while (featsIter.hasNext()) { 
			PackedFeature pFeat = (PackedFeature) featsIter.next();
			String feature	= pFeat.feat;
			String value	= pFeat.value; 
			proxyStructure.setFeature(feature,value);
		} // end while over features 
	} // end features

    //=================================================================
    // COMPUTATION METHODS OVER (PACKED) LOGICAL FORMS
    //=================================================================

	/** 
		The method <i>getPackedNominalSort</i> returns the (stable) ontological sort of the given nominal. 
		
		@param nom The packed nominal
		@return String	The ontological sort
	*/ 

	protected String getPackedNominalSort (PackedNominal nom) { 
		TreeSet addedSorts = new TreeSet();
		ArrayList<PackedOntologicalSort> packedSorts = new ArrayList<PackedOntologicalSort>(Arrays.asList(nom.packedSorts));
		Iterator sortsIter = packedSorts.iterator();
		while (sortsIter.hasNext()) { 
			PackedOntologicalSort pSort = (PackedOntologicalSort) sortsIter.next();
			String sort = pSort.sort;
			if (!addedSorts.contains(sort)) { 
				addedSorts.add(sort);
			} // end if..check whether already added
		} // end while over sorts
		return (String) addedSorts.first();
	} // end getPackedNominalSort


	/** 
		The method <i>getDependentNominal</i> returns the first relation-type dependent under the 
		given head. The method returns <tt>null</tt> if there is no such dependent. If the type
		is specified as "", simply the first dependent is returned (if any). The method first cycles
		over the LF relations under the head; if none are found there, it tries the packing edges
		(provided there are any). Under a packing edge, the method returns the first target. 
		
		@param head		The head nominal
		@param relation	The type of dependent that is being looked for
		@param packedNoms	The map with the nominals in the packed logical form
		@return PackedNominal The (first) dependent under the head, of the given type 
	*/ 
	

	protected PackedNominal getDependentNominal (PackedNominal head, String relation, TreeMap packedNoms) { 
		PackedNominal result = null; 
		boolean depFound = false; 
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		if (relation.equals("") && relations.size() > 0) { 
			LFRelation rel = (LFRelation) relations.get(0); 
			result = (PackedNominal) packedNoms.get(rel.dep);
		} else { 
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext() && !depFound) { 
				LFRelation rel = (LFRelation) relsIter.next();
				if (rel.mode.equals(relation)) { 
					// Get the dependent nominal
					String depVar = rel.dep;
					if (packedNoms.containsKey(depVar)) { 
						result = (PackedNominal) packedNoms.get(depVar);
						depFound = true;
					} // end if check for availability of the nominal in the map
				} // end if.. check whether the right relation 
			} // end while over relations
			// If we still haven't found a dependent, cycle over
			// the packing edges
			if (!depFound) { 
				if (head.pEdges != null) { 
					ArrayList<PackingEdge> packingEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
					Iterator peIter = packingEdges.iterator();
					while (peIter.hasNext() && !depFound) { 
						PackingEdge packingEdge = (PackingEdge) peIter.next();
						if (packingEdge.mode.equals(relation)) { 
							ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
							if (targets.size() > 0) { 
								PackingNodeTarget target = targets.get(0);
								String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
								result = (PackedNominal) packedNoms.get(targetNV);
								log("[AbstractProxyFactory] Getting dependent nominal: scanning packing edge targets under head node ["+head.nomVar+"], taking the first of a total ["+targets.size()+"] with id ["+targetNV+"]");						
							} // end if.. check for available targets
						} // end if .. check for mode of packing edge
					} // end while
				} // end check for there being packing edges
			} // end if.. check whether to check the packing edges 
		} // end if..else check for type
		return result;
	} // end getDependentNominal


	/** 
	The method <i>getDependentNominals</i> returns all relation-type dependents under the 
	given head. The method returns <tt>null</tt> if there is no such dependent. If the type
	is specified as "", simply all dependents are returned (if any). The method first cycles
	over the LF relations under the head; if none are found there, it tries the packing edges
	(provided there are any). Under a packing edge, the method returns the first target. 

	@param head		The head nominal
	@param relation	The type of dependent that is being looked for
	@param packedNoms	The map with the nominals in the packed logical form
	@return TreeSet<PackedNominal> The dependents under the head, of the given type 
	 */ 
	protected TreeSet<PackedNominal> getDependentNominals (PackedNominal head, String relation, TreeMap packedNoms) { 
		log("getDependentNominals called.");
		TreeSet<PackedNominal> result = new TreeSet<PackedNominal>(
				new Comparator() {
					public int compare(Object a, Object b) {
						PackedNominal packednomA = (PackedNominal) a;
						PackedNominal packednomB = (PackedNominal) b;
						String aNV = packednomA.nomVar;
						String bNV = packednomB.nomVar;
					return aNV.compareTo(bNV);
				}
		});
		boolean depFound = false; 
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		if (relation.equals("") && relations.size() > 0) { 
			log("empty feature specification... getting all features then!");
			for (LFRelation currRel : relations) {
				log("adding current relation to the result set: " + packedNoms.get(currRel.dep));
				result.add((PackedNominal) packedNoms.get(currRel.dep)); 
				// OLD: LFRelation rel = (LFRelation) relations.get(0);
				// OLD: result = (PackedNominal) packedNoms.get(rel.dep);
			}
		} else { 
			log("checking for all ["+relation+"] features in a list of size ["+relations.size()+"]");
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext()) { // OLD  && !depFound) { 
				LFRelation rel = (LFRelation) relsIter.next();
				log("current feature (rel.mode): " + rel.mode);
				if (rel.mode.equals(relation)) { 
					log("current feature equals the specified feature: " + relation);
					// Get the dependent nominal
					String depVar = rel.dep;
					if (packedNoms.containsKey(depVar)) { 
						log("packedNoms containsKey ["+depVar+"] -> adding ["+packedNoms.get(depVar)+"] to the result set.");
						result.add((PackedNominal) packedNoms.get(depVar));
						depFound = true;
					} // end if check for availability of the nominal in the map
				} // end if.. check whether the right relation 
			} // end while over relations
			// If we still haven't found a dependent, cycle over
			// the packing edges
			if (!depFound || head.pEdges != null) { 
				log("we have not yet found the feature we're looking for: cycling over the packing edges now!");
				if (head.pEdges != null) { 
					ArrayList<PackingEdge> packingEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
					Iterator peIter = packingEdges.iterator();
					while (peIter.hasNext() && !depFound) { 
						PackingEdge packingEdge = (PackingEdge) peIter.next();
						if (packingEdge.mode.equals(relation)) { 
							ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
							if (targets.size() > 0) { 
								log("Getting dependent nominal: scanning packing edge targets under head node ["+head.nomVar+"], taking all ["+targets.size()+"] targets. Does that make sense? (hz)");						
								for (PackingNodeTarget currTarget : targets) {
									String targetNV = currTarget.pnId.substring(0,currTarget.pnId.indexOf("_PN"));
									PackedNominal target = (PackedNominal) packedNoms.get(targetNV); 
									if (target != null) { 
										log("Adding packed nominal target ["+target.nomVar+"] for packing edge type ["+packingEdge.mode+"]");
										result.add(target);
									} else {
										log("Failure to find packed nominal target ["+targetNV+"]");
									}
								} // end for
							} // end if.. check for available targets
						} // end if .. check for mode of packing edge
					} // end while
				} // end check for there being packing edges
			} // end if.. check whether to check the packing edges 
		} // end if..else check for type
		if (result.isEmpty()) result = null;
		return result;
	} // end getDependentNominal


/** 
	The method <i>getFeatureValue</i> returns the value of a given feature of a given head. 
	The method returns <tt>null</tt> if there is no such dependent. If the type
	is specified as "", simply the first feature is returned (if any). 

	@param head		The head nominal
	@param feature	The type of feature that is being looked for
	@return String The value of the (first) feature of the given type 
	 */ 
	protected String getFeatureValue (PackedNominal head, String feature) { 
		String result = null; 
		boolean depFound = false;
		ArrayList<PackedFeature> features = new ArrayList<PackedFeature>(Arrays.asList(head.feats));
		if (feature.equals("") && features.size() > 0) { 
			PackedFeature feat = (PackedFeature) features.get(0);
			// result = (String) packedNoms.get(feat);
			result = (String) feat.value;
		} else { 
			Iterator featsIter = features.iterator();
			while (featsIter.hasNext() && !depFound) { 
				PackedFeature feat = (PackedFeature) featsIter.next();
				if (feat.feat.equals(feature)) { 
					// Get the value
					String val = feat.value;
					result = val;
					depFound = true;
				} // end if.. check whether the right feature
			} // end while over all features
		} // end if..else check for type
		return result;
	} // end getFeatureValue


	/**
		The method <i>getDependentNomVars</i> recursively descends to gather the nominal variables governed by the head. We assume that a head governs itself. 
		
		@param	head The nominal from which to descend
		@param  packedNoms The index into the nominals in the packed logical form
		@return TreeSet The set of nominals dependent on the head

	*/ 
	protected TreeSet getDependentNomVars (PackedNominal head, TreeMap packedNoms) { 
		// Initialize the result
		TreeSet result = new TreeSet();
		// Add the nomvar of the head
		result.add(head.nomVar);
		// Cycle over the relations
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		if (relations.size() > 0) { 
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext()) { 
				// Get the next relation
				LFRelation rel = (LFRelation) relsIter.next();
				// Descend down the dependent and get its governed nomvars
				PackedNominal dep = (PackedNominal) packedNoms.get(rel.dep);
				result.addAll(getDependentNomVars(dep,packedNoms));
			} // end while over relations
		} // end if check for availability of dependents		
		// Return the result
		return result;
	} // end getDependentNomVars


	/**
		The method <i>getFirstDependentType</i> returns the type of the first dependent under the head. 
		If the given head has no dependent, the empty string "" is returned. 
	*/ 
	
	protected String getFirstDependentType (PackedNominal head) { 
		String result = "";
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		if (relations.size() > 0) { 
			LFRelation rel = (LFRelation) relations.get(0); 
			result = rel.mode;
		} // end if check for availability of dependents
		return result;
	} // end getFirstDependentType
	
	
	
	/**
	 The method <i>getDependentType</i> returns the type of dependent relation under the given head, for the given dependent nominal variable. 
	 If the method needs to cycle over packing edges, the assumption is made that these are all of the same type -- only one type is returned. 
	 Any "Subject" type relations are being omitted. 
	
	 @param head		The head under which to look for the dependent
	 @param depNomVar	The nominal variable of the dependent 
	 @returns String	The type of dependent relation, if any; if none, <tt>null</tt> is returned
	*/ 

	protected String getDependentType (PackedNominal head, String depNomVar) { 
		String result = null;
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
		Iterator<LFRelation> relsIter = relations.iterator();
		while (relsIter.hasNext()) { 
			LFRelation rel = relsIter.next(); 
			if (rel.dep.equals(depNomVar) && !rel.mode.equals("Wh-Restr")  && !rel.mode.equals("Subject")) { 
				result = rel.mode;
				break; 
			} // end if.. check for dependent found	
		} // end if check for availability of dependents
		if (result == null) { 
			if (head.pEdges != null) { 
				ArrayList<PackingEdge> peEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
				Iterator peIter = peEdges.iterator();
				while (peIter.hasNext()) { 
					PackingEdge packingEdge = (PackingEdge) peIter.next();
					ArrayList<PackingNodeTarget> targets = new ArrayList<PackingNodeTarget>(Arrays.asList(packingEdge.targets));
					if (targets.size() > 0) { 
						for(Iterator targetIter = targets.iterator(); targetIter.hasNext(); ) { 
							// get the target
							PackingNodeTarget target = (PackingNodeTarget) targetIter.next();
							String targetNV = target.pnId.substring(0,target.pnId.indexOf("_PN"));
							if (targetNV.equals(depNomVar) && !packingEdge.mode.equals("Subject") && !packingEdge.mode.equals("Wh-Restr")) { 
								result = packingEdge.mode;
							} // end if..
						} // end for
					} // end if.. targets
				} // end while over packing edges
			} // end if.. check for packing edges
		} // end check for null intermediate result
		// Return the result
		log("Under head ["+head.nomVar+"] the dependent nomvar ["+depNomVar+"] is of type ["+result+"]");
		return result;		
	} // end getDependentType	
	
	protected String getSort (PackedNominal nom) {
		if (nom.packedSorts.length > 0) { 
			return nom.packedSorts[0].sort;
		} else { 
			return null;
		} // end if..else 
	} // end getSort
	

	/** 
		The method <i>hasDependent</i> returns a boolean indicating whether the nominal has a dependent of the given type. 
		
		@param head		The head nominal
		@param relation	The type being looked for
		@return boolean	Indicating whether the head has a relation of the given type
	*/ 
	protected boolean hasDependent (PackedNominal head, String relation) { 
		boolean result = false; 
		if (head != null && head.rels != null) { 
			ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(head.rels));
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext() && !result) { 
				LFRelation rel = (LFRelation) relsIter.next();
				// log("Checking relation ["+rel.mode+"]");
				if (rel.mode.equals(relation)) { 
						result = true;
				} // end if.. check whether the right relation 
			} // end while over relations
		} else { 
			System.err.println("ERROR [AbstractProxyFactory] cannot determine relation because head ["+head+"] or head.rels ["+head.rels+"]");
		} // end check for presence of relations
		// we may need to search for a packing edge instead
		if (!result && head.pEdges != null) { 
			ArrayList<PackingEdge> peEdges = new ArrayList<PackingEdge>(Arrays.asList(head.pEdges));
			Iterator peIter = peEdges.iterator();
			while (peIter.hasNext()) { 
				PackingEdge packingEdge = (PackingEdge) peIter.next();
				if (packingEdge.mode.equals(relation)) { 
					result = true; 
				} // end if.. check for mode
			} // end while over packing edges			
		} // end if.. check for packing edge
		return result;
	} // end hasDependent


	/**
	This is the method to be called for producing proxies. The method calls the factory production method, and ensures that all results are 
	properly passed upwards. 
	*/ 
	
	
	public ProxyFactoryResults produce (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) { 
		log("starting produce");
		proxyStructure = new LocalProxyStructure();
		ProxyFactoryResults results = produceProxies (nom, plf, packedNoms);
		log("produceProxies finished");
		if (results != null) { 
			// Provide the proxy structure in the results
			results.setProxyStructure(proxyStructure);
			// Add the relations to the result
			results.setPendingProxyRelations(pendingRels);
			// return the result
			return results;
		} else { 
			return null; 
		}
	} // end produce

	/** 
		Produces a proxy, or collection of proxies, starting from the given 
		nominal, using the packed logical form and the treemap-index into the nominals. 
		The method returns a results object including variables of nominals for which we should no 
		longer generate proxies, and the pending proxy relations
	*/
	protected abstract ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms);



	public void log (String m) { 
		if (logging) System.out.println("[ProxyFactory for "+rootSort+"] "+m);
	} 


} // end AbstractProxyFactory

