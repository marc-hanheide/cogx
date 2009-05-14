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
import org.cognitivesystems.comsys.general.CacheWrapper;

// ----------------------------------------------------------------
// JAVA imports
// ----------------------------------------------------------------
import java.util.TreeMap;
import java.util.TreeSet;

// ----------------------------------------------------------------
// LF imports
// ----------------------------------------------------------------
import org.cognitivesystems.repr.lf.autogen.LFPacking.*;


//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
The interface <b>ProxyFactory</b> produces a proxy, or a collection of 
proxies and proxy relations. The proxies are written to the working
memory of the binder, through the monitor associated with the 
factory. 

@version 071002 (started 071002)
@author  Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

//=================================================================
// INTERFACE DEFINITION 
//=================================================================

public interface ProxyFactory {

	/** Sets the cache with discourse referents to be used. */ 
	public void setDiscRefs (CacheWrapper dr); 



	/** Returns the ontological sort of the root nominal from which the factory produces its proxies. */ 
	public String getRootSort (); 

	/** Produces a proxy, or collection of proxies, starting from the given 
		nominal, using the packed logical form and the treemap-index into the nominals. 
		The method returns a results object including variables of nominals for which we should no 
		longer generate proxies, and the pending proxy relations
	*/
	public ProxyFactoryResults produce (PackedNominal nom, PackedLogicalForm plf, TreeMap packedNoms);







}
