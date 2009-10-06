//
//  EntityProxy.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference.proxyfactories;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.RelationProxy; 
import binder.utils.ProxyConstructor;

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryPointer;
import comsys.datastructs.lf.LFNominal; 
import comsys.datastructs.lf.LFRelation; 
import comsys.datastructs.lf.LogicalForm; 
import comsys.lf.utils.LFUtils;
import comsys.processing.reference.ProxyResults;

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.Iterator; 


/**
 Entity-proxy construction does 
 */ 


public class EntityProxy extends AbstractProxyFactory 
	{
			
		private String sort = "entity";
		
		public String getSort () { return sort; }
		
		public ProxyResults constructProxy (LogicalForm lf, CASTTime timestamp) { 
			assert lf.root.sort.equals("entity");
			ProxyResults prxs = new ProxyResults();
			// get the root
			LFNominal root = lf.root; 
			// create a base proxy, listing the proposition of the root as proposition, its sort as a sort. 
			WorkingMemoryPointer origin = ProxyConstructor.createWorkingMemoryPointer("comsys", lf.root.nomVar, "lf");
			PhantomProxy prx = ProxyConstructor.createNewPhantomProxy(origin, getEntityID(),  1.0f);
	//		ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature ("ling_label", root.prop.prop, timestamp));
			// Add the proximity of the (deictic)
			if (LFUtils.lfNominalHasFeature(root,"Proximity")) { 
				String proximity = LFUtils.lfNominalGetFeature(root,"Proximity");
				ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature ("ling_proximity", proximity, timestamp));			
			} // end if.. check for proximity
			
			// Add the resulting proxy
			prxs.addPhantomProxy(prx);
			// return the result
			return prxs; 
		} // end method
		
		
	
		
	} // end class




