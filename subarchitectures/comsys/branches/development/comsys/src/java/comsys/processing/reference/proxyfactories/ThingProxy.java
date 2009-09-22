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
import binder.autogen.core.Proxy; 
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.StringValue; 
import binder.autogen.specialentities.RelationProxy; 

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
			Proxy prx = createNewProxy(createWorkingMemoryPointer("comsys", lf.root.nomVar, "lf"), 1.0f);
			prx = addFeatureToProxy(prx, createSimpleFeature ("proposition", root.prop.prop));
			prx = addFeatureToProxy(prx, createSimpleFeature ("sort", root.sort));		
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
						
					}
				} else if (rel.mode.equals("Owner")) {
					ProxyResults prxresults = mapOwner(prx, lf, rel.head, rel.dep);		  
					
				} // end if..else
			} // end while
			// Add the resulting proxy
			prxs.addProxy(prx);
			// return the result
			return prxs; 
		} // end method
		
		/**
		 mapQualityModifier takes a modifier of sort q-color, q-shape, or q-size. The method creates 
		 a feature based on the sort (color, shape, size) and then adds the proposition as value. This
		 feature is then added to the proxy (provided as argument). The updated proxy is returned. 
		 
		 @param prx The proxy to be updated
		 @param mod The nominal containing the (quality) information to be added as feature
		 @returns Proxy The updated proxy
		 */ 
		
		private Proxy mapQualityModifier (Proxy prx, LFNominal mod) { 
			String feature = null;
			String value   = null; 
			if (mod.sort.equals("q-color")) { feature = "color"; value = mod.prop.prop; } 
			if (mod.sort.equals("q-shape")) { feature = "shape"; value = mod.prop.prop; } 
			if (mod.sort.equals("q-size"))  { feature = "size"; value = mod.prop.prop; } 
			assert value != null && feature != null; 
			prx = addFeatureToProxy (prx, createSimpleFeature(feature,value));
			return prx; 
		} // end mapQualityModifier
		
		
		/**
		 mapLocation
		 
		 */ 
		
		private ProxyResults mapLocation (Proxy head, LogicalForm lf, String headVar, String depVar) {  
			// initialize the results
			ProxyResults results = new ProxyResults();
			// fetch the nominal heading the modifier construction
			LFNominal modHead = LFUtils.lfGetNominal(lf, depVar);
			assert modHead != null; 
			// get the anchor under the location / whereto 
			LFRelation anchorRel = LFUtils.lfNominalGetRelation(modHead, "Anchor");
			assert anchorRel != null; 
			LFNominal anchor = LFUtils.lfGetNominal(lf, anchorRel.dep);
			assert anchor != null;
			// get the actual owner of the anchor: this is the landmark
			LFRelation ownerRel = LFUtils.lfNominalGetRelation(anchor, "Owner");
			assert ownerRel != null;
			LFNominal owner = LFUtils.lfGetNominal(lf, ownerRel.dep);
			assert owner != null; 
			// now construct the proxy for the owner
			ProxyResults ownerResults = null;
			if (owner.sort.equals("thing") | owner.sort.equals("e-place")) { 
				LogicalForm ownerLF = LFUtils.lfConstructSubtree(owner,lf);
				ownerResults = this.constructProxy(ownerLF);
			} // end if..	
			assert ownerResults != null;	
			Proxy ownerProxy = ownerResults.getProxyByNom(owner.nomVar); 
			assert ownerProxy != null; 
			// construct the relation between the head and the owner directly
			AddressValue[] sources = createAddressValueArray(createAddressValue(head.entityID, 1.0f));
			AddressValue[] targets = createAddressValueArray(createAddressValue(ownerProxy.entityID, 1.0f));
			RelationProxy rprx = createNewRelationProxy(createWorkingMemoryPointer("comsys", headVar, "lf"), 1.0f, sources, targets);
			// construct the results
			results.addProxies(ownerResults);
			results.addRelationProxies(ownerResults);
			results.addRelationProxy(rprx);
			// return the results
			return results;
		} // end mapLocation
		
		/**
		 mapOwner 
		 
		 */ 
		
		
		private ProxyResults mapOwner (Proxy head, LogicalForm lf, String headVar, String depVar) { 
			// Construct the proxy for the owner
			LFNominal owner = LFUtils.lfGetNominal(lf,depVar);
			Proxy prx = createNewProxy(createWorkingMemoryPointer("comsys", depVar, "lf"), 1.0f);
			prx = addFeatureToProxy(prx, createSimpleFeature("proposition", owner.prop.prop));
			prx = addFeatureToProxy(prx, createSimpleFeature("sort", owner.sort));		
			// Construct the relation between the head and the owner
			AddressValue[] sources = createAddressValueArray(createAddressValue(head.entityID, 1.0f));
			AddressValue[] targets = createAddressValueArray(createAddressValue(prx.entityID, 1.0f));
			RelationProxy rprx = createNewRelationProxy(createWorkingMemoryPointer("comsys", headVar, "lf"), 1.0f, sources, targets);		
			addFeatureToProxy(rprx, createSimpleFeature("label", "OwnedBy"));
			// Construct the results;
			ProxyResults results = new ProxyResults();
			results.addProxy(prx);
			results.addRelationProxy(rprx);
			return results;
		} // end mapOwner
		
		
		
		
	} // end class




