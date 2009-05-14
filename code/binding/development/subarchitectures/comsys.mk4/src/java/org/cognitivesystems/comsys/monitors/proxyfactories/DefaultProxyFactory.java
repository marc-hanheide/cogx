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
import BindingFeatures.DebugString;
import BindingFeatures.SourceID;


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
The class <b>DefaultProxyFactory</b> implements the default mapping 
for producing a proxy, or a collection of proxies and proxy relations. 
The proxies are provided as local proxy structures. 

@version 080616 (started 071002)
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class DefaultProxyFactory 
	extends AbstractProxyFactory
	
{

    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================


    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public DefaultProxyFactory () { 
		super();
	} // end

//	public DefaultProxyFactory (ComSysBindingMonitor m) {
//		super(m);
//	} // end 

    //=================================================================
    // COMPUTATION METHODS
    //=================================================================


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
	protected ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 
	
	{  
		log("Running the default proxy factory.");
		ProxyFactoryResults result = new ProxyFactoryResults();

		// Add the nominal variable as DebugString feature
		proxyStructure.setFeature("DiscRef",_discRefsAccess.getDiscRef(nom.nomVar));

		// Add the proposition to the proxy structure 
		proxyStructure.setFeature("Proposition",nom.prop.prop);
		
		// Get the proposition of the nominal and add as concept
		proxyStructure.setFeature("Concept",nom.prop.prop);

		// Add the features to the nominal
		ArrayList<PackedFeature> packedFeats = new ArrayList<PackedFeature>(Arrays.asList(nom.feats));
		Iterator featsIter = packedFeats.iterator();
		addFeatures(featsIter);

		// Create the relations
		ArrayList<LFRelation> relations = new ArrayList<LFRelation>(Arrays.asList(nom.rels));
		Iterator relsIter = relations.iterator();
		while (relsIter.hasNext()) { 
			LFRelation rel = (LFRelation) relsIter.next();
			PendingProxyRelation ppr = new PendingProxyRelation();
			ppr.headNomVar = nom.nomVar; 
			ppr.relMode = rel.mode;
			ppr.depNomVar = rel.dep;
			pendingRels.addElement(ppr);
			proxyStructure.addRelation(ppr.headNomVar,ppr.depNomVar,ppr.relMode);
		} // end while over relations

		// Make explicit what nominals have been added
		TreeSet addedNoms = new TreeSet();
		addedNoms.add(nom.nomVar);
		result.setAddedNominals(addedNoms);			
		proxyStructure.addCoveredNodes(addedNoms);
		// Return the results
		return result;
	} // end produceProxies

    //=================================================================
    // I/O METHODS
    //=================================================================

	public void log (String msg) { 
		if (logging) System.out.println("[DefaultProxyFactory] "+msg);
	} // end log

} // end DefaultProxyFactory


