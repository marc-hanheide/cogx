//=================================================================
// Copyright (C) 2008 Geert-Jan M. Kruijff (gj@dfki.de)
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
// COMSYS imports
// ----------------------------------------------------------------
import org.cognitivesystems.comsys.monitors.AbstractProxyFactory;
import org.cognitivesystems.comsys.monitors.PendingProxyRelation;
import org.cognitivesystems.comsys.monitors.ProxyFactoryResults;
import org.cognitivesystems.comsys.monitors.LocalProxyStructure;


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
import org.cognitivesystems.repr.lf.utils.LFUtils;


public class CognitionProxyFactory 
	extends AbstractProxyFactory
{

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public CognitionProxyFactory () { 
		super();
		logging = false;
		rootSort = "cognition";
	} // end



	/** Produces a proxy, or collection of proxies, starting from the given 
		nominal, using the packed logical form and the treemap-index into the nominals. 
		The method returns a result structure including variables of nominals for which we should no 
		longer generate proxies, and the relations that should be introduced.
		
		@param nom The packed nominal from which proxy production should start
		@param plf The packed logical form in which the nominal appears
		@param packedNoms The map with nominal variables indexing into the packed logical form
		@return ProxyFactoryResults A set with nominal variables of packed nominals for which proxies have been produced
		@see org.cognitivesystems.comsys.monitors.AbstractProxyFactory#getRelProxiesTable		 
		
	*/
	public ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 
	{  
		proxyStructure = new LocalProxyStructure();
		// Initialize the result structures
		ProxyFactoryResults result = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		
		// -------------------------------------------------------------
		// HANDLING NOMINAL NODE / LOGICAL FORM CONTENT
		// -------------------------------------------------------------
	
		// Get the verbal proposition
		String verb = nom.prop.prop;

		String mood = LFUtils.plfGetFeatureValue(nom,"Mood");
		if (mood != null) { 
			if (mood.equals("int")) { 
				InterrogativeMoodFactory qFactory = new InterrogativeMoodFactory();
				ProxyFactoryResults qResult = qFactory.produceProxies(nom,plf,packedNoms);
				// Update the excludes
				excludes.addAll(qResult.getExcludes());
				// Update the relations
				pendingRels.addAll(qResult.getPendingProxyRelations());
				proxyStructure.addRelations(qResult.getPendingProxyRelations().iterator());				
			} // end if.. check for interrogative
		} 
		
		// Differentiate by proposition how to handle all the relations
		if (verb.equals("know") ||
			verb.equals("understand") ||
			verb.equals("mean") ||
			verb.equals("want") ||
			verb.equals("need") ||
			verb.equals("have") ||
			verb.equals("memorize") ||
			verb.equals("assume") ||
			verb.equals("hope") ||			
			verb.equals("suppose") ||			
			verb.equals("think") ||			
			verb.equals("believe") ||			
			verb.equals("imagine") ||						
			verb.equals("bet") ||									
			verb.equals("got")
			) 
		{
			CognitionKnowProxyFactory knowPF = new CognitionKnowProxyFactory();
			knowPF.setDiscRefs(_discRefsAccess);
			ProxyFactoryResults knowResult = knowPF.produceProxies(nom,plf,packedNoms);
			// Fuse with the resulting proxy structure features
			proxyStructure.fuse(knowResult.getProxyStructure());
			// Update the excludes
			excludes.addAll(knowResult.getExcludes());
			// Update the relations
			pendingRels.addAll(knowResult.getPendingProxyRelations());
			proxyStructure.addRelations(knowResult.getPendingProxyRelations().iterator());
			// Create the interrogative structures, if necessary

		} else { 
			System.err.println("Cognition Factory is being called for an unknown verb ["+verb+"]");
			// System.exit(0);
		} // end if..else 
		
		// -------------------------------------------------------------
		// HANDLING RESULTS
		// -------------------------------------------------------------
		
		// Make explicit what nominals have been added
		TreeSet addedNoms = new TreeSet();
		addedNoms.add(nom.nomVar);
		result.setAddedNominals(addedNoms);		
		proxyStructure.addCoveredNodes(addedNoms);	
		proxyStructure.addContentStatus("intentional");
		// Set the excludes
		result.setProxyStructure(proxyStructure);
		result.setExcludes(excludes);
		result.setPendingProxyRelations(pendingRels);
		// Return the results
		log("Returning the following proxy structure:\n"+proxyStructure.toString());
		return result;
	} // end produceProxies


} // end PerceptionProxyFactory
