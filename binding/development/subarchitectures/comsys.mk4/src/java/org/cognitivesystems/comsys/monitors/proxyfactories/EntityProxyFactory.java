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

package org.cognitivesystems.comsys.monitors.proxyfactories;

//=================================================================
// IMPORTS
//=================================================================

// ----------------------------------------------------------------
// BINDING imports
// ----------------------------------------------------------------
import binding.common.BindingComponentException;
import BindingFeatures.Concept;
import BindingFeatures.DebugString;

// ----------------------------------------------------------------
// CAST imports
// ----------------------------------------------------------------
import cast.architecture.subarchitecture.SubarchitectureProcessException;

// ----------------------------------------------------------------
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.monitors.AbstractProxyFactory;
import org.cognitivesystems.comsys.monitors.ComSysBindingMonitor;
import org.cognitivesystems.comsys.monitors.LocalProxyStructure;
import org.cognitivesystems.comsys.monitors.PendingProxyRelation;
import org.cognitivesystems.comsys.monitors.ProxyFactoryResults;


// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.ArrayList;
import java.util.Arrays;
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
The class <b>EntityProxyFactory</b> implements the  mapping 
for producing a proxy, or a collection of proxies and proxy relations, 
for RESTRICTED-ENTITY type nominals. 

The mapping consists skipping proxy generation (null result). 


@version 071107 (started 071107)
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class EntityProxyFactory 
	extends AbstractProxyFactory
	
{

    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================




    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public EntityProxyFactory () { 
		super();
		rootSort = "entity";
	} // end


    //=================================================================
    // COMPUTATION METHODS
    //=================================================================

	/** Avoids producing a proxy for this type of nominal. 
		
		@param nom The packed nominal from which proxy production should start
		@param plf The packed logical form in which the nominal appears
		@param packedNoms The map with nominal variables indexing into the packed logical form
		@return ProxyFactoryResults null in this case, to ensure no proxies are generated
		@see org.cognitivesystems.comsys.monitors.AbstractProxyFactory#getRelProxiesTable		 
		
	*/
	public ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 
	
	{  
		proxyStructure = new LocalProxyStructure();
		ProxyFactoryResults result = new ProxyFactoryResults();
		addBasicContent(nom);
		TreeSet excludes = new TreeSet();
		if (nom.prop.prop.startsWith("wh")) { 
			proxyStructure.updateFeature("Concept","WH:"+"?");
			proxyStructure.addContentStatus("intentional");
		} else { 
			// Create the relations
			ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(nom.rels));
			Iterator relsIter = relations.iterator();
			while (relsIter.hasNext()) { 
				LFRelation rel = (LFRelation) relsIter.next();
				// if (rel.mode.equals("Property")) { 
				if (rel.mode.equals("Modifier")) {
					// Get the dependent nominal
					PackedNominal depNom = (PackedNominal) packedNoms.get(rel.dep);
					String propertySort = null; 
					// Check whether it is a property, or a location
					if (depNom.packedSorts.length > 0) { 
						propertySort = depNom.packedSorts[0].sort;
					} 
					if (propertySort != null) { 
						if (propertySort.equals("m-location")) { 
							ProxyFactoryResults mapResult = mapLocation(nom,packedNoms);
							excludes.addAll(mapResult.getExcludes());
							pendingRels.addAll(mapResult.getPendingProxyRelations());		
							proxyStructure.addRelations(mapResult.getPendingProxyRelations().iterator());	
						} else { 
							ProxyFactoryResults propResult = mapProperty(rel.dep,packedNoms);
							// Fuse with the resulting proxy structure features
							proxyStructure.fuse(propResult.getProxyStructure());
							log("New proxy structure, fused with property: \n"+proxyStructure.toString());
							// Set the excludes
							excludes.addAll(propResult.getExcludes());
							// Add the relations
							pendingRels.addAll(propResult.getPendingProxyRelations());
							proxyStructure.addRelations(propResult.getPendingProxyRelations().iterator());
						} // end if..else check for material property or location
					} // check that there is a sort
				} else { 
					log("Putting in a new LFRelation-based relation of type ["+rel.mode+"] under E-Location ["+nom.nomVar+"]");
					PendingProxyRelation ppr = new PendingProxyRelation();
					ppr.headNomVar = nom.nomVar; 
					ppr.relMode = rel.mode;
					ppr.depNomVar = rel.dep;
					pendingRels.addElement(ppr);
					proxyStructure.addRelation(ppr);
				} // end if..else check for property relation
			} // end while over relations
		} // end if..else check for question words
		
		
		
		TreeSet addedNoms = new TreeSet();
		addedNoms.add(nom.nomVar);
		proxyStructure.addCoveredNodes(addedNoms);	
		result.setExcludes(excludes);
		result.setAddedNominals(addedNoms);	
		result.setProxyStructure(proxyStructure);
		result.setPendingProxyRelations(pendingRels);
		return result;
	} // end produceproxies
	

	/** 
		The method <i>mapLocation</i> creates the amodal representation of a thing's location. We need to deal with two types of constructions: 
		ones in which we have an anchoring region relative to an object ("to the left of the yellow star") versus a direction relative to an 
		object ("left of the yellow star"). The first construction involves a Modifier (m-location) with a dependent Anchor (e-location) which in turn 
		has a dependent Owner (thing). The second construction involves a Modifier (m-location) with a dependent Anchor (thing). In the first construction, 
		it is the e-location which provides the label for the proxy-relation, whereas in the second construction it is the m-location. 
		
		@param object		The nominal of the object
		@param packedNoms	The index map for the nominals in the packed logical form
		@return ProxyFactoryResults	Results with a treeset with nominal variables to be excluded from further processing, and a location relation
	*/ 
	protected ProxyFactoryResults mapLocation (PackedNominal object, TreeMap packedNoms) 
	{ 
		// Initialize the result structures
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Get the modifier
		PackedNominal modifier = getDependentNominal(object, "Modifier", packedNoms); 
		if (modifier != null) { 
			// Establish the proposition of the modifier
			String modProp = modifier.prop.prop;
			// Exclude the modifier from further processing
			excludes.add(modifier.nomVar);
			// Get the anchors dependents
			TreeSet<PackedNominal> anchors = getDependentNominals(modifier,"Anchor",packedNoms); 			
			// Iterate over each anchor, check whether it is an e-location or a not; 
			// if it is an e-location, create a proxy relation of label e-location proposition, to its Owner object
			// if it is not an e-location, create a proxy relation of label modProp, to the Anchor object
			for (Iterator anchorsIter = anchors.iterator(); anchorsIter.hasNext(); ) { 	
				PackedNominal anchor = (PackedNominal) anchorsIter.next();
				String anchorSort = null; 
				// Check whether it is a property, or a location
				if (anchor.packedSorts.length > 0) { 
					anchorSort = anchor.packedSorts[0].sort;
				} 
				// Differentiate between constructions; note that the ELSE also applies to NULL sorts!
				if (anchorSort.equals("e-location")) { 
					String amodalSpatialRelation = anchor.prop.prop;
					PackedNominal owner = getDependentNominal(anchor,"Owner",packedNoms);
					if (owner != null) { 
						PendingProxyRelation spatialRel = new PendingProxyRelation(object.nomVar,owner.nomVar,amodalSpatialRelation);
						pendingRelations.addElement(spatialRel);
						log("Added a location relation under ["+object.nomVar+"] of type ["+amodalSpatialRelation+"] to ["+owner.nomVar+"]");
					} else { 
						PendingProxyRelation spatialRel = new PendingProxyRelation(object.nomVar,anchor.nomVar,"Location");
						pendingRelations.addElement(spatialRel);
						log("Added a location relation under ["+object.nomVar+"] of type ["+amodalSpatialRelation+"] to ["+anchor.nomVar+"]");						
					} 
					// end if..check for anchor
					// check whether the location should be processed further ... as it may have embedded modifiers
					// excluding it would mean no recursive descent after this point
					excludes.add(anchor.nomVar);				
				} else { 
					String amodalSpatialRelation = modProp;
					PendingProxyRelation spatialRel = new PendingProxyRelation(object.nomVar,anchor.nomVar,amodalSpatialRelation);
					pendingRelations.addElement(spatialRel);
					excludes.add(modifier.nomVar);
					log("Added a location relation under ["+object.nomVar+"] of type ["+amodalSpatialRelation+"] to ["+anchor.nomVar+"]");
				} // end if..else check for construction type
			} // end for over anchors
		} else {
			log("No modifier found for locative modification of an object"); 
		} // end if..else check for the modifier
		// Set the results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);
		return results;
	} // end mapLocation






	/**
		The method <i>mapProperty</i> creates the amodal representation of material properties of a thing. The resulting property features are
		provided in the local proxy structure stored in the returned ProxyFactoryResults. 
		
		@param depVar	The nominal variable of the object's property dependent
		@param packedNoms The index map for the nominals in the packed logical form
		@return TreeSet	A treeset with nominal variables to be excluded from further processing
	*/ 
	protected ProxyFactoryResults mapProperty (String depVar, TreeMap packedNoms) 
	{ 
		ProxyFactoryResults results = new ProxyFactoryResults();
		LocalProxyStructure resultPRX = new LocalProxyStructure();
		TreeSet excludes = new TreeSet();
		if (packedNoms.containsKey(depVar)) { 
			PackedNominal depNom = (PackedNominal) packedNoms.get(depVar);
			// Check for its sort; we assume single sort, so take the first one
			if (depNom.packedSorts.length > 0) { 
				String propertySort = depNom.packedSorts[0].sort;
				if(propertySort.equals("q-colour") || propertySort.equals("q-color")) {
					resultPRX.setFeature("Colour",depNom.prop.prop);
				} else if (propertySort.equals("q-size")) { 
					resultPRX.setFeature("Size",depNom.prop.prop);
				} // end if..else checking for types of properties
				excludes.add(depVar);
			} // end if..check for sorts present
		} else { 
		} // end if..else check for availability of dependent	
		// Set the proxy structure
		results.setProxyStructure(resultPRX);
		
		
		log("Returning mapped property proxy structure: \n"+resultPRX.toString());
		
		// Set the excludes		
		results.setExcludes(excludes);
		return results; 
	} // end mapProperty




} // end EntityProxyFactory

