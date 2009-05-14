package org.cognitivesystems.comsys.general;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.Cache;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.CacheMapping;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.IndexAssociation;
import org.cognitivesystems.comsys.data.CacheElement;
import org.cognitivesystems.repr.lf.utils.LFUtils;
import org.omg.CORBA.Any;

public class CacheWrapper {
	
	Cache cache;

	public CacheWrapper(Cache cache) {
		this.cache = cache;
		
		if (cache.content1 == null) {
			cache.content1 = new Any[0];
		}
		if (cache.content2==null) {
			cache.content2 = new Any[0];
		}
		if (cache.mapping == null) {
			cache.mapping = new CacheMapping();
			cache.mapping.associations = new IndexAssociation[0];
		}
	}
	
	public Cache getCache() {
		return cache;
	}
	
	public int getSize() {
		return cache.content1.length;
	}
	
	/**
	 * Return the discourse referent associated to a given nomVar, if there is one.
	 * else, return "none"
	 * @param nomVar
	 * @return
	 */
	public String getDiscRef(String nomVar) {
		String result = "none";
		for (int i=0; i < cache.content2.length ; i++) {
			String nomVar2 = cache.content2[i].extract_string();
			if (nomVar2.equals(nomVar)) {
				for (int j=0; j < cache.mapping.associations.length ; j++) {
					int index = cache.mapping.associations[j].id2[0];
					if (i == index) {
						result = cache.content1[cache.mapping.associations[j].id1[0]].extract_string();
						break;
					}
				}
			}
		}
		// System.out.println("CacheWrapper: checking for discourse referent for nomvar ["+nomVar+"], result is ["+result+"]");
		return result;
	}
	
	/**
	 * Return the nomVar associated with a given discourse referent, if there is one.
	 * else, return "none"
	 * @param nomVar
	 * @return
	 */
	public String getNomVar(String nomVar) {
		String result = "none";
		for (int i=0; i < cache.content1.length ; i++) {
			String nomVar2 = cache.content1[i].extract_string();
			if (nomVar2.equals(nomVar)) {
				for (int j=0; j < cache.mapping.associations.length ; j++) {
					int index = cache.mapping.associations[j].id1[0];
					if (i == index) {
						result = cache.content1[cache.mapping.associations[j].id2[0]].extract_string();
						break;
					}
				}
			}
		}
		// System.out.println("CacheWrapper: checking for discourse referent for nomvar ["+nomVar+"], result is ["+result+"]");
		return result;
	}




	public void addCacheAssociation (String element1, String element2) {

		int content1Size = cache.content1.length;
		int content2Size = cache.content2.length;
		int assocSize = cache.mapping.associations.length;
		cache.content1 = (Any[]) LFUtils.resizeArray(cache.content1, content1Size+1);
		cache.content2 = (Any[]) LFUtils.resizeArray(cache.content2, content2Size+1);
		cache.mapping.associations = (IndexAssociation[]) LFUtils.resizeArray
		(cache.mapping.associations, assocSize+1);
		cache.content1[content1Size] = new CacheElement(element1);
		cache.content2[content2Size] = new CacheElement(element2);
		cache.mapping.associations[assocSize] = new IndexAssociation();
		cache.mapping.associations[assocSize].id1 = new int[1];
		cache.mapping.associations[assocSize].id1[0] = content1Size ;
		cache.mapping.associations[assocSize].id2 = new int[1];
		cache.mapping.associations[assocSize].id2[0] = content2Size ;
		cache.mapping.associations[assocSize].relType = "SINGULAR";

	}
}
