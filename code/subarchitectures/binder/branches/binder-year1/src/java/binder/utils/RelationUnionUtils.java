package binder.utils;

import java.util.HashMap;
import java.util.Vector;

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.core.logging.ComponentLogger;

import binder.autogen.core.Feature;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.RelationUnion;

public class RelationUnionUtils {

	
	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	// flag to activate logging
	public static boolean LOGGING = true;

	private static Logger logger = ComponentLogger.getLogger(ProbabilityUtils.class);


	public static RelationUnion specifyUnionSourceAndTarget (RelationUnion union, UnionConfiguration config) {
		log("start relation processing");
		
		HashMap<String, String> unionForProxy = getUnionForProxyHashMap(config);
		
		Feature usource= getUnionSource(union, unionForProxy);
		Feature utarget = getUnionTarget(union, unionForProxy);
	
		String entityID = union.entityID;
		ProbabilityDistribution distribution = union.distribution;
		Proxy[] includedProxies = union.includedProxies;
		Feature[] features = union.features;
		float probExists = union.probExists;
		CASTTime timeStamp = union.timeStamp;
		Feature psource = union.psource;
		Feature ptarget = union.ptarget;

		
		RelationUnion newUnion = new RelationUnion
			(entityID, probExists, timeStamp, features, distribution, 
					includedProxies, psource, ptarget, usource, utarget);

		return newUnion;
	}
	
	
	private static HashMap<String, String> getUnionForProxyHashMap (UnionConfiguration config) {
		HashMap<String, String> unionForProxy = new HashMap<String, String>();
		for (int j = 0; j < config.includedUnions.length ; j++) {
			Union curUnion = config.includedUnions[j];
			for (int k = 0 ; k < curUnion.includedProxies.length; k++) {
				unionForProxy.put(curUnion.includedProxies[k].entityID, curUnion.entityID);
			}
		}
		return unionForProxy;
	}
	
	
	private static Feature getUnionSource (RelationUnion union, HashMap<String, String> unionForProxy) {
		
		Vector<AddressValue> salterValues = new Vector<AddressValue>();
		for (int i = 0 ; i < union.psource.alternativeValues.length; i++) {
			String sourceId = ((AddressValue)union.psource.alternativeValues[i]).val;
			
			if (unionForProxy.containsKey(sourceId)) {
				float independentProb = union.psource.alternativeValues[i].independentProb;
				CASTTime stimestamp = union.psource.alternativeValues[i].timeStamp;
				String sval = unionForProxy.get(sourceId);
				AddressValue newSource = new AddressValue(independentProb,stimestamp, sval);
				salterValues.add(newSource);
                log("bound source of " + union.entityID + " to " + newSource.val);
			}
		}
		
		AddressValue[] salternativeValues = new AddressValue[salterValues.size()];
		salternativeValues = salterValues.toArray(salternativeValues);

		Feature usource = new Feature("source", salternativeValues);

		return usource;
	}
	
	
	private static Feature getUnionTarget (RelationUnion union, HashMap<String, String> unionForProxy) {
		
		Vector<AddressValue> talterValues = new Vector<AddressValue>();

		for (int i = 0 ; i < union.ptarget.alternativeValues.length; i++) {
			String targetId = ((AddressValue)union.ptarget.alternativeValues[i]).val;
			
			if (unionForProxy.containsKey(targetId)) {
				float independentProb = union.ptarget.alternativeValues[i].independentProb;
				CASTTime ttimeStamp = union.ptarget.alternativeValues[i].timeStamp;
				String tval = unionForProxy.get(targetId);
				AddressValue newTarget = new AddressValue(independentProb,ttimeStamp, tval);
				talterValues.add(newTarget);
                log("bound target of " + union.entityID + " to " + newTarget.val);
			}
		}
		
		AddressValue[] talternativeValues = new AddressValue[talterValues.size()];
		talternativeValues = talterValues.toArray(talternativeValues);

		Feature utarget = new Feature("target", talternativeValues);
		
		return utarget;
	}
	

	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 
	
	

	public static void log(String s) {
		if (LOGGING)
			logger.debug("[RelationUnionUtils] " + s);
	}

	public static void errlog(String s) {
		if (ERRLOGGING)
			logger.debug("[RelationUnionUtils] " + s);
	}
	
}
