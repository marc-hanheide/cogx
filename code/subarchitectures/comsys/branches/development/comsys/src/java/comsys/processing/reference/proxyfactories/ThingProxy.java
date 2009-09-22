//
//  ThingProxy.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference.proxyfactories;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

import binder.autogen.core.Proxy; 

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.lf.LogicalForm; 
import comsys.processing.reference.ProxyFactory;
import comsys.processing.reference.ProxyResults;

/**
	Thing-proxy construction does 
*/ 


public class ThingProxy 
		extends AbstractProxyFactory 
{

	private String sort = "thing";
	
	public String getSort () { return sort; }
	
	public ProxyResults constructProxy (LogicalForm lf) { 
		assert lf.root.sort.equals("thing");
		ProxyResults prxs = new ProxyResults();
		// get the root
		LFNominal root = lf.root; 
		Proxy prx = createNewProxy("comsys", 1.0f);
		Iterator relsIter = LFUtils.lfNominalGetRelations(root); 
		while (relsIter.hasNext()) { 
			LFRelation rel = relsIter.next();
			if (rel.mode.equals("Modifier")) { 
				LFNominal mod = LFUtils.lfGetNominal(lf,rel.dep);
				// check whether we have a quality
				if (mod.sort.startsWith("q-")) {
					prx = mapQualityModifier(prx,mod);
				} 
			} // end if
		} // end while
		// Add the resulting proxy
		prxs.addProxy(prx, lf.nomVar);
		// return the result
		return prxs; 
	} // end method
	
	
	private Proxy mapQualityModifier (Proxy prx, LFNominal mod) { 
		String feature = null;
		String value   = null; 
		if (mod.sort.equals("q-color")) { feature = "color"; value = mod.prop.prop; } 
		if (mod.sort.equals("q-shape")) { feature = "shape"; value = mod.prop.prop; } 
		if (mod.sort.equals("q-size"))  { feature = "size"; value = mod.prop.prop; } 
		assert value != null && feature != null; 
		StringValue strValue = createStringValue (value, 1.0f);
		Feature prxFeature = createFeatureWithUniqueFeatureValue (feature, strValue);
		prx = addFeatureToProxy (prx, prxFeature);
		return prx; 
	} // end mapQualityModifier
	
	
} // end class
