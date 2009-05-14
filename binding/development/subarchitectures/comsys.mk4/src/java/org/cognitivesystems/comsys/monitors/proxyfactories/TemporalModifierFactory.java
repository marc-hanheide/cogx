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
// BINDING imports
// ----------------------------------------------------------------
import binding.common.BindingComponentException;

// ----------------------------------------------------------------
// CAST imports
// ----------------------------------------------------------------
import cast.architecture.subarchitecture.SubarchitectureProcessException;

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

/**
	The class <b>TemporalModifierFactory</b> provides a method for producing time-modifier structures for verbs. 
	Every relation produced here will have "intentional" content status
	
	@started 080703
	@version 080725
	@author	 Geert-Jan M. Kruijff (gj@dfki.de)
*/ 

public class TemporalModifierFactory
	extends AbstractProxyFactory 
 {


	public ProxyFactoryResults produceProxies (PackedNominal action, PackedLogicalForm plf, TreeMap packedNoms) { 
		// Initialize the result structures
		ProxyFactoryResults results = new ProxyFactoryResults();
		TreeSet excludes = new TreeSet();
		Vector pendingRelations = new Vector();
		// Get the location dependent(s) of the object
		TreeSet<PackedNominal> modifiers = getDependentNominals(action,"Modifier",packedNoms); 
		for (Iterator modsIter = modifiers.iterator(); modsIter.hasNext(); ) { 
			PackedNominal modifier = (PackedNominal) modsIter.next();
			if (modifier != null) { 
				String modSort = null; 
				// Get the sort of the modifier
				if (modifier.packedSorts.length > 0) { modSort = modifier.packedSorts[0].sort; } 
				// Ensure we have a sort, 
				if (modSort != null) { 				
					if (modSort.equals("m-time-point") || modSort.equals("m-time")) { 
						// check whether the modifier has an Anchor, or whether it is an atomic point spec
						if (hasDependent(modifier,"Anchor")) { 
							// in a while
							PackedNominal anchor = getDependentNominal(modifier,"Anchor",packedNoms);
							PendingProxyRelation timeRel = new PendingProxyRelation(action.nomVar, anchor.nomVar, "Time-Point:"+modifier.prop.prop);
							timeRel.addContentStatus("intentional");				
							pendingRelations.addElement(timeRel);
							excludes.add(modifier.nomVar);	
						} else if (hasDependent(modifier, "Event")) { 
							// since yesterday
							PackedNominal event = getDependentNominal(modifier,"Event",packedNoms);
							PendingProxyRelation timeRel = new PendingProxyRelation(action.nomVar, event.nomVar, "Time-Point:"+modifier.prop.prop);
							timeRel.addContentStatus("intentional");											
							pendingRelations.addElement(timeRel);
							excludes.add(modifier.nomVar);								
						} else { 		
							PendingProxyRelation timeRel = new PendingProxyRelation(action.nomVar, modifier.nomVar,"Time-Point");
							timeRel.addContentStatus("intentional");											
							pendingRelations.addElement(timeRel);
						} // end if..else check for anchor
					} else if (modSort.equals("m-time-interval")) { 		
						// for a while
						if (hasDependent(modifier,"Anchor")) { 
							PackedNominal anchor = getDependentNominal(modifier,"Anchor", packedNoms);
							PendingProxyRelation timeRel = new PendingProxyRelation(action.nomVar, anchor.nomVar, "Time-Interval:"+modifier.prop.prop);
							timeRel.addContentStatus("intentional");											
							pendingRelations.addElement(timeRel);
							excludes.add(modifier.nomVar);
						} 
					} else if (modSort.equals("m-time-sequence")) { 
						if (hasDependent(modifier,"Event")) { 
							PackedNominal event = getDependentNominal(modifier,"Event",packedNoms);
							PendingProxyRelation timeRel = new PendingProxyRelation(action.nomVar,event.nomVar,"Time-Sequence:"+modifier.prop.prop);
							timeRel.addContentStatus("intentional");				
							pendingRelations.addElement(timeRel);
							excludes.add(modifier.nomVar);
						} else { 		
							PendingProxyRelation timeRel = new PendingProxyRelation(action.nomVar,modifier.nomVar,"Time-Sequence");
							timeRel.addContentStatus("intentional");								
							pendingRelations.addElement(timeRel);
						} // end if..else
					} 
				} // end if..check for sort
			} else { 
			} // end if..else check for availability of dependent
		} // end for over locations
		// Set the results
		results.setExcludes(excludes);
		results.setPendingProxyRelations(pendingRelations);
		return results;
	} // end mapTimeModifier
}
