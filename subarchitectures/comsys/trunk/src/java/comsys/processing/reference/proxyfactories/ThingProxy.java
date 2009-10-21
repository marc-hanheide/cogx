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
 Thing-proxy construction does 
 */ 


public class ThingProxy extends AbstractProxyFactory 
	{
		
		private String sort = "thing";
		
		public String getSort () { return sort; }
		
		public ProxyResults constructProxy (LogicalForm lf, CASTTime timestamp) { 
			assert lf.root.sort.equals("thing");
			ProxyResults prxs = new ProxyResults();
			// get the root
			LFNominal root = lf.root; 
			// create a base proxy, listing the proposition of the root as proposition, its sort as a sort. 
			WorkingMemoryPointer origin = ProxyConstructor.createWorkingMemoryPointer("comsys", lf.root.nomVar, "lf");
			PhantomProxy prx = ProxyConstructor.createNewPhantomProxy(origin, getEntityID(),  1.0f);
			ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature ("ling_label", root.prop.prop, timestamp));
		//	ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature ("sort", root.sort, timestamp));		
			// iterate over the relations, and add the content
			
			if (LFUtils.lfNominalHasFeature(root,"Proximity")) { 
				String proximity = LFUtils.lfNominalGetFeature(root,"Proximity");
				ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature ("ling_proximity", proximity, timestamp));			
			} // end if.. check for proximity
			
			Iterator<LFRelation> relsIter = LFUtils.lfNominalGetRelations(root); 
			while (relsIter.hasNext()) { 
				LFRelation rel = relsIter.next();
				if (rel.mode.equals("Modifier")) { 
					LFNominal mod = LFUtils.lfGetNominal(lf,rel.dep);
					// check whether we have a quality
					if (mod.sort.startsWith("q-")) {
						prx = mapQualityModifier(prx,mod, timestamp);
					} else if (mod.sort.equals("m-location")) {		
						
					}
				} else if (rel.mode.equals("Owner")) {
					ProxyResults prxresults = mapOwner(prx, lf, rel.head, rel.dep, timestamp);		  
					
				} // end if..else
			} // end while
			// Add the resulting proxy
			prxs.addPhantomProxy(prx);
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
		
		private PhantomProxy mapQualityModifier (PhantomProxy prx, LFNominal mod, CASTTime timestamp) { 
			String feature = null;
			String value   = null; 
			if (mod.sort.equals("q-color")) { feature = "ling_attribute"; value = mod.prop.prop; } 
			if (mod.sort.equals("q-shape")) { feature = "ling_attribute"; value = mod.prop.prop; } 
			if (mod.sort.equals("q-size"))  { feature = "ling_attribute"; value = mod.prop.prop; } 
			assert value != null && feature != null; 
			ProxyConstructor.addFeatureToProxy (prx, createSimpleFeature(feature,value, timestamp));
			return prx; 
		} // end mapQualityModifier
		
		
		/**
		 mapLocation
		 
		 */ 
		
		private ProxyResults mapLocation (PhantomProxy head, LogicalForm lf, String headVar, String depVar, CASTTime timestamp) {  
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
				ownerResults = this.constructProxy(ownerLF, timestamp);
			} // end if..	
			assert ownerResults != null;	
			PhantomProxy ownerProxy = ownerResults.getPhantomProxyByNom(owner.nomVar); 
			assert ownerProxy != null; 
			// construct the relation between the head and the owner directly
			AddressValue val1 = ProxyConstructor.createAddressValue(head.entityID, 1.0f);
			ProxyConstructor.setTimeStamp(val1, timestamp);
			AddressValue[] sources = createAddressValueArray(val1);
			AddressValue val2 = ProxyConstructor.createAddressValue(ownerProxy.entityID, 1.0f);
			ProxyConstructor.setTimeStamp(val2, timestamp);
			AddressValue[] targets = createAddressValueArray(val2);
			WorkingMemoryPointer origin = ProxyConstructor.createWorkingMemoryPointer("comsys", headVar, "lf");
			RelationProxy rprx = ProxyConstructor.createNewRelationProxy(origin, getEntityID(), 1.0f, sources, targets);
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
		
		 
		private ProxyResults mapOwner (PhantomProxy head, LogicalForm lf, String headVar, String depVar, CASTTime timestamp) { 
			// Construct the proxy for the owner
			LFNominal owner = LFUtils.lfGetNominal(lf,depVar);
			WorkingMemoryPointer origin = ProxyConstructor.createWorkingMemoryPointer("comsys", depVar, "lf");
			PhantomProxy prx = ProxyConstructor.createNewPhantomProxy(origin, getEntityID(), 1.0f);
			ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature("proposition", owner.prop.prop, timestamp));
			ProxyConstructor.addFeatureToProxy(prx, createSimpleFeature("sort", owner.sort, timestamp));		
			// Construct the relation between the head and the owner
			AddressValue val = ProxyConstructor.createAddressValue(head.entityID, 1.0f);
			ProxyConstructor.setTimeStamp(val, timestamp);
			AddressValue[] sources = createAddressValueArray(val);
			AddressValue val2 = ProxyConstructor.createAddressValue(prx.entityID, 1.0f);
			ProxyConstructor.setTimeStamp(val2, timestamp);
			AddressValue[] targets = createAddressValueArray(val2);
			WorkingMemoryPointer origin2 = ProxyConstructor.createWorkingMemoryPointer("comsys", headVar, "lf");
			RelationProxy rprx = ProxyConstructor.createNewRelationProxy(origin2, getEntityID(), 1.0f, sources, targets);		
			ProxyConstructor.addFeatureToProxy(rprx, createSimpleFeature("label", "OwnedBy", timestamp));
			// Construct the results;
			ProxyResults results = new ProxyResults();
			results.addPhantomProxy(prx);
			results.addRelationProxy(rprx);
			return results;
		} // end mapOwner
		
		
		
		
	} // end class




