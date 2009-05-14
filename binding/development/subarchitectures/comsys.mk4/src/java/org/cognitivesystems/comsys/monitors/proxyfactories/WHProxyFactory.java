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
The class <b>AnimateProxyFactory</b> implements the  mapping 
for producing a proxy, or a collection of proxies and proxy relations, 
for ANIMATE type nominals. 

The proxies are written to the working memory of the binder, 
through the monitor associated with the  factory. 

@version 080710
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class WHProxyFactory 
	extends AbstractProxyFactory
	
{

    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

	public String questionArgument; 
	

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public WHProxyFactory () { 
		super();
		rootSort = "specifier";
		questionArgument = "?";
	} // end


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
	
	public ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 
	
	{  
		questionArgument = "?";
		proxyStructure = new LocalProxyStructure();
		ProxyFactoryResults result = new ProxyFactoryResults();
		addBasicContent(nom);
		TreeSet excludes = new TreeSet();

		if (hasDependent(nom,"Scope")) { 
			PackedNominal scope = getDependentNominal(nom,"Scope",packedNoms);
			questionArgument=questionArgument+scope.prop.prop;
		} 

		proxyStructure.updateFeature("Concept","WH:"+questionArgument);
		proxyStructure.addContentStatus("intentional");

		TreeSet addedNoms = new TreeSet();
		addedNoms.add(nom.nomVar);
		proxyStructure.addCoveredNodes(addedNoms);	
		result.setExcludes(excludes);
		result.setAddedNominals(addedNoms);	
		result.setProxyStructure(proxyStructure);
		result.setPendingProxyRelations(pendingRels);		
		return result;
	} // end produceproxies
	




} // end WHProxyFactory

