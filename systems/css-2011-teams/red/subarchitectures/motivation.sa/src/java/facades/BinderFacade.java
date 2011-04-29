package facades;

import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.CASTHelper;
import castutils.castextensions.WMView;
import de.dfki.lt.tr.beliefs.data.Belief;
import de.dfki.lt.tr.beliefs.slice.sitbeliefs.dBelief;

public class BinderFacade extends CASTHelper {
	private static final String BINDER_SA = "binder";

	static private BinderFacade singleton;

	WMView<dBelief> proxies;
	ManagedComponent component;

	/**
	 * @param component
	 */
	public BinderFacade(ManagedComponent component) {
		super(component);
		this.component = component;
		this.proxies = WMView.create(component, dBelief.class, "binder");
	}

	/**
	 * get the singleton reference
	 * 
	 * @param component
	 * @return the singleton
	 * @throws CASTException
	 */
	public static BinderFacade get(ManagedComponent component) {
		if (singleton == null) {
			singleton = new BinderFacade(component);
			singleton.start();
		}
		return singleton;
	}

	public void start() {
		try {
			proxies.start();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

//	/**
//	 * search for a specific feature in the entitiy
//	 * 
//	 * @param entity
//	 * @param featureLabel
//	 * @return the alternativeFeatures
//	 */
//	public List<FeatureValue> getFeatureValue(Belief entity, String featureLabel) {
//		assert (entity.content instanceof CondIndependentDistribs);
//		CondIndependentDistribs cid = (CondIndependentDistribs) entity.content;
//		ProbDistribution pd = cid.distribs.get(featureLabel);
//		if (pd == null)
//			return new LinkedList<FeatureValue>();
//
//		assert (pd instanceof BasicProbDistribution);
//
//		List<FeatureValue> result = new LinkedList<FeatureValue>();
//
//		for (FeatureValueProbPair f : ((FeatureValues) ((BasicProbDistribution) pd).values).values) {
//			result.add(f.val);
//		}
//		return result;
//	}
//
//	/**
//	 * iterates all unions to search for features that match the given id
//	 * 
//	 * @param featureLabel
//	 *            the label to search for
//	 * @return a map of union-ids (String) as keys and FeatureValues as value
//	 */
//
//	public Map<String, FeatureValue> findFeaturesInUnion(String featureLabel) {
//		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
//
//		// List<FeatureValue> features;
//
//		for (Belief union : proxies.values()) {
//
//			debug("check belief " + union.id);
//			List<FeatureValue> fvl = getFeatureValue(union, featureLabel);
//			if (fvl.size() > 0) {
//				result.put(union.id, fvl.get(0));
//			}
//		}
//		return result;
//
//	}
//
//	/**
//	 * look up proxy using its ID
//	 * 
//	 * @param id
//	 * @return the found proxy or null if non was found
//	 */
//	public Belief getProxy(String id) {
//		for (Belief proxy : new HashSet<Belief>(proxies.values())) {
//			if (proxy.id.equals(id)) {
//				return proxy;
//			}
//		}
//		return null;
//	}
//
//	public Map<WorkingMemoryAddress, Belief> findRelationBySrc(String src) {
//		return findRelationByAttribute(src, RelationElement0.value);
//	}
//
//	public Map<WorkingMemoryAddress, Belief> findRelationByTarget(String src) {
//		return findRelationByAttribute(src, RelationElement1.value);
//	}
//
//	public Map<WorkingMemoryAddress, Belief> findRelationByAttribute(
//			String src, String attr) {
//		Map<WorkingMemoryAddress, Belief> result = new HashMap<WorkingMemoryAddress, Belief>();
//		for (Entry<WorkingMemoryAddress, Belief> e : new HashSet<Entry<WorkingMemoryAddress, Belief>>(
//				proxies.entrySet())) {
//			Belief rp = e.getValue();
//			if (rp.type.equals(TypeRelation.value)) {
//				StringValue fv = (StringValue) getFeatureValue(rp, attr).get(0);
//				if (fv.equals(src)) {
//					result.put(e.getKey(), rp);
//				}
//			}
//		}
//		return result;
//
//	}
//
//	public Map<WorkingMemoryAddress, Belief> findRelations(String targetOrSrc,
//			String relationType) {
//		log("look for all relations having target or source equal to "
//				+ targetOrSrc + " and being of type " + relationType);
//		Map<WorkingMemoryAddress, Belief> result = new HashMap<WorkingMemoryAddress, Belief>();
//		result.putAll(findRelationBySrc(targetOrSrc));
//		result.putAll(findRelationByTarget(targetOrSrc));
//		if (relationType.length() > 0) {
//			log("check relation type");
//			for (Entry<WorkingMemoryAddress, Belief> e : result.entrySet()) {
//				List<FeatureValue> fvl = getFeatureValue(e.getValue(),
//						relationType);
//				log("fvl.size() == " + fvl.size());
//				if (fvl.size() == 0) {
//					continue;
//				}
//			}
//		}
//		return result;
//	}
//
//	public Map<WorkingMemoryAddress, Belief> getRelatedProxies(Belief prx,
//			String relationType) {
//		Map<WorkingMemoryAddress, Belief> result = new HashMap<WorkingMemoryAddress, Belief>();
//
//		Map<WorkingMemoryAddress, Belief> selectedRelations = findRelations(
//				prx.id, relationType);
//		log("getRelatedProxies: selectedRelations = "
//				+ selectedRelations.size());
//		for (Entry<WorkingMemoryAddress, Belief> e : selectedRelations
//				.entrySet()) {
//			String sourceId = ((StringValue) getFeatureValue(e.getValue(),
//					RelationElement0.value).get(0)).val;
//			String targetId = ((StringValue) getFeatureValue(e.getValue(),
//					RelationElement1.value).get(0)).val;
//			debug("getRelatedProxies: sourceID = " + sourceId);
//			debug("getRelatedProxies: targetID = " + targetId);
//			WorkingMemoryAddress sourceAddr = new WorkingMemoryAddress(
//					sourceId, BINDER_SA);
//			WorkingMemoryAddress targetAddr = new WorkingMemoryAddress(
//					targetId, BINDER_SA);
//			// the respective sources and target proxies should be in the maps
//			Belief tmpProxy = proxies.get(sourceAddr);
//			// if we find them and they are different from the proxy used to
//			// query, add them to the results
//			if (tmpProxy != null && !tmpProxy.equals(prx))
//				result.put(sourceAddr, tmpProxy);
//			tmpProxy = proxies.get(targetAddr);
//			if (tmpProxy != null && !tmpProxy.equals(prx))
//				result.put(targetAddr, tmpProxy);
//		}
//
//		return result;
//	}
//
//	/**
//	 * extract the place_id from a plac proxy
//	 * 
//	 * @param prx
//	 *            the proxy to analyse
//	 * @return the place_id of the proxy or -1 if it does not contain a valid
//	 *         place_id
//	 */
//	public long getPlaceIdFromProxy(Belief prx) {
//		List<FeatureValue> fvl = getFeatureValue(prx, FeatPlaceId.value);
//		if (fvl.size() > 0) {
//			FeatureValue fv = fvl.get(0);
//			if (fv instanceof StringValue) {
//				IntegerValue placeID = (IntegerValue) fv;
//				return placeID.val;
//			}
//		}
//
//		return -1;
//
//	}
//
//	public dBelief findPlaceProxy(Place p) {
//		for (dBelief prx : new HashSet<dBelief>(proxies.values())) {
//			long place_id = getPlaceIdFromProxy(prx);
//			if (place_id == p.id) {
//				return prx;
//			}
//		}
//		return null;
//	}

	public Belief<dBelief> getBelief(String id) {
        WorkingMemoryAddress wma = new WorkingMemoryAddress(id, BINDER_SA);
		return Belief.create(dBelief.class,proxies.get(wma));
	}

}
