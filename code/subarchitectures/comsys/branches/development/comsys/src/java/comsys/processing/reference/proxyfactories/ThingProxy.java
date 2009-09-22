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

import binder.autogen.core.Feature; 
import binder.autogen.core.OriginInfo; 
import binder.autogen.core.Proxy; 
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.StringValue; 

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.lf.LFNominal; 
import comsys.datastructs.lf.LFRelation; 
import comsys.datastructs.lf.LogicalForm; 
import comsys.lf.utils.LFUtils;
import comsys.processing.reference.ProxyFactory;
import comsys.processing.reference.ProxyResults;

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.Iterator; 


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
		// create a base proxy, listing the proposition of the root as proposition, its sort as a sort. 
		Proxy prx = createNewProxy(new OriginInfo ("comsys", lf.root.nomVar, "lf"), 1.0f);
		prx.addFeatureToProxy(prx, createFeatureWithUniqueFeatureValue ("proposition", createStringValue (root.prop.prop, 1.0f)));
		prx.addFeatureToProxy(prx, createFeatureWithUniqueFeatureValue ("sort", createStringValue (root.sort, 1.0f)));		
		// iterate over the relations, and add the content
		Iterator<LFRelation> relsIter = LFUtils.lfNominalGetRelations(root); 
		while (relsIter.hasNext()) { 
			LFRelation rel = relsIter.next();
			if (rel.mode.equals("Modifier")) { 
				LFNominal mod = LFUtils.lfGetNominal(lf,rel.dep);
				// check whether we have a quality
				if (mod.sort.startsWith("q-")) {
					prx = mapQualityModifier(prx,mod);
				} else if (mod.sort.equals("m-location")) {		

					
			} else if (rel.mode.equals("Owner")) {
					ProxyResults prxresults = mapOwner(prx, lf, rel.head, rel.dep);		  
					
				
				
			} else {
							  
			} // end if..else
		} // end while
		// Add the resulting proxy
		prxs.addProxy(prx, lf.root.nomVar);
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
	
	
	
	private ProxyResults mapLocation (LogicalForm lf, String nomVar) {  
	
	
	
	} // end mapLocation
	
	
	private ProxyResults mapOwner (Proxy head, LogicalForm lf, String headVar, String depVar) { 
		// Construct the proxy for the owner
		LFNominal owner = LFUtils.lfGetNominal(lf,nomVar);
		Proxy prx = createNewProxy(new OriginInfo ("comsys", depVar, "lf"), 1.0f);
		prx.addFeatureToProxy(prx, createFeatureWithUniqueFeatureValue ("proposition", createStringValue (owner.prop.prop, 1.0f)));
		prx.addFeatureToProxy(prx, createFeatureWithUniqueFeatureValue ("sort", createStringValue (owner.sort, 1.0f)));		
		// Construct the relation between the head and the owner
		AddressValue[] sources = createAddressValueArray(createNewAddressValue(head.entityID, 1.0f));
		AddressValue[] targets = createAddressValueArray(createNewAddressValue(prx.entityID, 1.0f));
		RelationProxy rprx = createNewRelationProxy(new OriginInfo("comsys", headVar, "lf"), 1.0f, sources, targets);		
		// Construct the results;
		ProxyResults results = new ProxyResults();
		results.addProxy(prx);
		results.addRelationProxy(rprx);
		return results;
	} // end mapOwner
	
	private AddressValue[] createAddressValueArray (AddressValue addr) { 	
		AddressValue[] addressarray = new AddressValue[1];
		addressarray[0] = addr;
		return addressarray;
	} // end method
		
		
} // end class




