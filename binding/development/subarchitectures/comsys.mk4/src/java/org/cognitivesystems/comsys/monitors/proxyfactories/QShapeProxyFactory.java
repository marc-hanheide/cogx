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
The class <b>QShapeToProxyFactory</b> implements the  mapping 
for producing a proxy, or a collection of proxies and proxy relations, 
for Q-SHAPE type nominals. 

The mapping consists in ensuring any "args" are skipped (they are dealt with at a higher level). 


@version 071018 (started 071018)
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class QShapeProxyFactory 
	extends AbstractProxyFactory
	
{

    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================




    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

	public QShapeProxyFactory () { 
		super();
		rootSort = "q-shape";
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
	protected ProxyFactoryResults produceProxies (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms) 

	{  
		ProxyFactoryResults results = new ProxyFactoryResults();
		proxyStructure = new LocalProxyStructure();
		proxyStructure.setFeature("Concept","?entity");
		proxyStructure.setFeature("Shape",nom.prop.prop);							
		results.setProxyStructure(proxyStructure);
		results.setExcludes(new TreeSet());
		// Make explicit what nominals have been added
		TreeSet addedNoms = new TreeSet();
		addedNoms.add(nom.nomVar);
		results.setAddedNominals(addedNoms);		
		proxyStructure.addCoveredNodes(addedNoms);	
		results.setProxyStructure(proxyStructure);			
		return results;
	} // end produceProxies
	

} // end MWhereToProxyFactory

