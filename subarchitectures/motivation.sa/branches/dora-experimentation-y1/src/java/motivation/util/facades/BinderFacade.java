package motivation.util.facades;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import motivation.util.castextensions.CASTHelper;
import motivation.util.castextensions.WMEntrySet;
import motivation.util.castextensions.WMView;
import SpatialData.Place;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.specialentities.RelationProxy;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;

public class BinderFacade extends CASTHelper {
	private static final String BINDER_SA = "binder";

	static private BinderFacade singleton;

	WMView<UnionConfiguration> unionConfigurations;
	WMView<Proxy> proxies;
	WMView<RelationProxy> relations;
	ManagedComponent component;

	/**
	 * @param component
	 */
	public BinderFacade(ManagedComponent component) {
		super(component);
		this.component = component;
		this.unionConfigurations = WMView.create(component,
				UnionConfiguration.class);
		this.proxies = WMView.create(component, Proxy.class, "binder");
		this.relations = WMView
				.create(component, RelationProxy.class, "binder");
	}

	/**
	 * get the singleton reference
	 * 
	 * @param component
	 * @return the singleton
	 * @throws CASTException
	 */
	public static BinderFacade get(ManagedComponent component)
			throws CASTException {
		if (singleton == null) {
			singleton = new BinderFacade(component);
			singleton.start();
		}
		return singleton;
	}

	public void start() {
		try {
			unionConfigurations.start();
			proxies.start();
			relations.start();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	/**
	 * search for a specific feature in the entitiy
	 * 
	 * @param entity
	 * @param featureLabel
	 * @return the alternativeFeatures
	 */
	public List<FeatureValue> getFeatureValue(PerceivedEntity entity,
			String featureLabel) {
		List<FeatureValue> result = new LinkedList<FeatureValue>();
		for (Feature f : entity.features) {
			debug("  check feature: " + f.featlabel);
			if (f.featlabel.equals(featureLabel)) {
				for (FeatureValue av : f.alternativeValues) {
					result.add(av);
				}
				break;
			}
		}
		return result;
	}

	/**
	 * iterates all unions to search for features that match the given id
	 * 
	 * @param featureLabel
	 *            the label to search for
	 * @return a map of union-ids (String) as keys and FeatureValues as value
	 */

	public Map<String, FeatureValue> findFeaturesInUnion(String featureLabel) {
		Map<String, FeatureValue> result = new HashMap<String, FeatureValue>();
		if (unionConfigurations.size() < 1)
			return result;
		List<FeatureValue> features;
		Union[] unions = ((UnionConfiguration) unionConfigurations.values()
				.iterator().next()).includedUnions;
		if (unions == null)
			return result;

		debug("unions:  " + unions.length);
		debug("proxies: " + proxies.size());
		for (Union union : unions) {
			debug("check union " + union.entityID);
			features = getFeatureValue(union, featureLabel);
			for (FeatureValue fv : features) {
				result.put(union.entityID, fv);
			}
		}
		return result;

	}

	/**
	 * look up union using its ID
	 * 
	 * @param id
	 * @return the found union or null if non was found
	 */
	public Union getUnion(String id) {
		if (unionConfigurations.size() < 1)
			return null;
		Union[] unions = ((UnionConfiguration) unionConfigurations.values()
				.iterator().next()).includedUnions;
		for (Union union : unions) {
			if (union.entityID.equals(id)) {
				return union;
			}
		}
		return null;
	}

	/**
	 * look up union using its ID
	 * 
	 * @param id
	 * @return the found union or null if non was found
	 */
	public Map<String, Union> getUnions() {
		Map<String, Union> result = new HashMap<String, Union>();

		if (unionConfigurations.size() < 1)
			return result;
		Union[] unions = ((UnionConfiguration) unionConfigurations.values()
				.iterator().next()).includedUnions;
		for (Union union : unions) {
			result.put(union.entityID, union);
		}
		return result;
	}

	/**
	 * look up proxy using its ID
	 * 
	 * @param id
	 * @return the found proxy or null if non was found
	 */
	public Proxy getProxy(String id) {
		for (Proxy proxy : new HashSet<Proxy>(proxies.values())) {
			if (proxy.entityID.equals(id)) {
				return proxy;
			}
		}
		return null;
	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelationBySrc(String src) {
		Map<WorkingMemoryAddress, RelationProxy> result = new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, RelationProxy> e : new HashSet<Entry<WorkingMemoryAddress, RelationProxy>>(relations.entrySet())) {
			RelationProxy rp = e.getValue();
			if (((AddressValue) rp.source.alternativeValues[0]).val.equals(src))
				result.put(e.getKey(), rp);
		}
		return result;

	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelationByTarget(
			String target) {
		Map<WorkingMemoryAddress, RelationProxy> result = new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, RelationProxy> e : new HashSet<Entry<WorkingMemoryAddress, RelationProxy>>(relations.entrySet())) {
			RelationProxy rp = e.getValue();
			if (((AddressValue) rp.target.alternativeValues[0]).val
					.equals(target))
				result.put(e.getKey(), rp);
		}
		return result;
	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelations(
			String targetOrSrc, String relationType) {
		log("look for all relations having target or source equal to "
				+ targetOrSrc + " and being of type " + relationType);
		Map<WorkingMemoryAddress, RelationProxy> result = new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, RelationProxy> e : new HashSet<Entry<WorkingMemoryAddress, RelationProxy>>(relations.entrySet())) {
			RelationProxy rp = e.getValue();
			// check if the label is the one we search for; if it's not, skip
			// going
			if (relationType.length() > 0) {
				log("check relation type");
				List<FeatureValue> fvl = getFeatureValue(rp, relationType);
				log("fvl.size() == " + fvl.size());
				if (fvl.size() == 0) {
					continue;
				}
			}
			log("looking for target and source");
			Feature src=rp.source;
			Feature tgt=rp.target;
			if (src!=null && tgt!=null) {
				FeatureValue[] tav = rp.target.alternativeValues;
				FeatureValue[] sav = rp.source.alternativeValues;
				if (tav.length>0 && sav.length>0) {
					log("comparing source and target with " + targetOrSrc);
					if (((AddressValue) sav[0]).val.equals(targetOrSrc)) {
						result.put(e.getKey(), rp);
					}
					else if (((AddressValue) tav[0]).val.equals(targetOrSrc)) {
						result.put(e.getKey(), rp);
					}
					
				} else {
					println("problem: source or target has now values... source=="+sav.length+ ", target=="+tav.length);
				}
			} else {
				println("problem: source or target is null... source=="+src+ ", target=="+tgt);
			}
		}
		return result;
	}

	public Map<WorkingMemoryAddress, Proxy> getRelatedProxies(Proxy prx,
			String relationType) {
		Map<WorkingMemoryAddress, Proxy> result = new HashMap<WorkingMemoryAddress, Proxy>();

		Map<WorkingMemoryAddress, RelationProxy> selectedRelations = findRelations(
				prx.entityID, relationType);
		log("getRelatedProxies: selectedRelations = "
				+ selectedRelations.size());
		for (Entry<WorkingMemoryAddress, RelationProxy> e : selectedRelations
				.entrySet()) {
			String sourceId = ((AddressValue) e.getValue().source.alternativeValues[0]).val;
			String targetId = ((AddressValue) e.getValue().target.alternativeValues[0]).val;
			debug("getRelatedProxies: sourceID = " + sourceId);
			debug("getRelatedProxies: targetID = " + targetId);
			WorkingMemoryAddress sourceAddr = new WorkingMemoryAddress(
					sourceId, BINDER_SA);
			WorkingMemoryAddress targetAddr = new WorkingMemoryAddress(
					targetId, BINDER_SA);
			// the respective sources and target proxies should be in the maps
			Proxy tmpProxy = proxies.get(sourceAddr);
			// if we find them and they are different from the proxy used to
			// query, add them to the results
			if (tmpProxy != null && !tmpProxy.equals(prx))
				result.put(sourceAddr, tmpProxy);
			tmpProxy = proxies.get(targetAddr);
			if (tmpProxy != null && !tmpProxy.equals(prx))
				result.put(targetAddr, tmpProxy);
		}

		return result;
	}

	/**
	 * extract the place_id from a plac proxy
	 * 
	 * @param prx
	 *            the proxy to analyse
	 * @return the place_id of the proxy or -1 if it does not contain a valid
	 *         place_id
	 */
	public long getPlaceIdFromProxy(Proxy prx) {
		List<FeatureValue> fvl = getFeatureValue(prx, "place_id");
		if (fvl.size() > 0) {
			FeatureValue fv = fvl.get(0);
			if (fv instanceof StringValue) {
				StringValue placeID = (StringValue) fv;
				return Long.parseLong(placeID.val);
			}
		}

		return -1;

	}

	public Proxy findPlaceProxy(Place p) {
		for (Proxy prx : new HashSet<Proxy>(proxies.values())) {
			long place_id = getPlaceIdFromProxy(prx);
			if (place_id == p.id) {
				return prx;
			}
		}
		return null;
	}

}
