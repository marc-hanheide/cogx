package motivation.util.facades;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import motivation.util.castextensions.WMEntrySet;
import motivation.util.castextensions.WMEntrySet.ChangeHandler;
import Ice.ObjectImpl;
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
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;

public class BinderFacade {
	private static final String BINDER_SA = "binder";

	static private BinderFacade singleton;

	WMEntrySet unionConfigurations;
	WMEntrySet proxies;
	WMEntrySet relations;
	ManagedComponent component;

	/**
	 * @param component
	 */
	public BinderFacade(ManagedComponent component) {
		super();
		this.component = component;
		this.unionConfigurations = WMEntrySet.create(component,
				UnionConfiguration.class);
		this.proxies = WMEntrySet.create(component, Proxy.class);
		this.relations = WMEntrySet.create(component, RelationProxy.class);
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
		}
		return singleton;
	}


	public void start() {
		unionConfigurations.start();
		proxies.start();
		relations.start();
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
			component.debug("  check feature: " + f.featlabel);
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

		component.debug("unions:  " + unions.length);
		component.debug("proxies: " + proxies.size());
		for (Union union : unions) {
			component.debug("check union " + union.entityID);
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
		for (ObjectImpl o : proxies.values()) {
			Proxy proxy = (Proxy) o;
			if (proxy.entityID.equals(id)) {
				return proxy;
			}
		}
		return null;
	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelationBySrc(String src) {
		Map<WorkingMemoryAddress, RelationProxy> result = new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> e : relations.entrySet()) {
			RelationProxy rp = (RelationProxy) e.getValue();
			if (((AddressValue) rp.source.alternativeValues[0]).val.equals(src))
				result.put(e.getKey(), rp);
		}
		return result;

	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelationByTarget(
			String target) {
		Map<WorkingMemoryAddress, RelationProxy> result = new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> e : relations.entrySet()) {
			RelationProxy rp = (RelationProxy) e.getValue();
			if (((AddressValue) rp.target.alternativeValues[0]).val
					.equals(target))
				result.put(e.getKey(), rp);
		}
		return result;
	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelations(
			String targetOrSrc, String relationType) {
		Map<WorkingMemoryAddress, RelationProxy> result = new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> e : relations.entrySet()) {
			RelationProxy rp = (RelationProxy) e.getValue();
			// check if the label is the one we search for; if it's not, skip going
			if (relationType.length()>0) {
				if (!rp.features[0].featlabel.equals(relationType)) {
					continue;
				}
			}
			if (((AddressValue) rp.target.alternativeValues[0]).val
					.equals(targetOrSrc))
				result.put(e.getKey(), rp);
			else if (((AddressValue) rp.source.alternativeValues[0]).val
					.equals(targetOrSrc))
				result.put(e.getKey(), rp);
		}
		return result;
	}

	public Map<WorkingMemoryAddress, Proxy> getRelatedProxies(
			Proxy prx, String relationType) {
		Map<WorkingMemoryAddress, Proxy> result = new HashMap<WorkingMemoryAddress, Proxy>();

		Map<WorkingMemoryAddress, RelationProxy> selectedRelations = findRelations(prx.entityID, relationType);

		for (Entry<WorkingMemoryAddress, RelationProxy> e : selectedRelations
				.entrySet()) {
			String sourceId = ((AddressValue) e.getValue().source.alternativeValues[0]).val;
			String targetId = ((AddressValue) e.getValue().target.alternativeValues[0]).val;
			WorkingMemoryAddress sourceAddr = new WorkingMemoryAddress(
					sourceId, BINDER_SA);
			WorkingMemoryAddress targetAddr = new WorkingMemoryAddress(
					targetId, BINDER_SA);
			// the respective sources and target proxies should be in the maps
			Proxy tmpProxy = (Proxy) proxies.get(sourceAddr);
			// if we find them and they are different from the proxy used to
			// query, add them to the results
			if (tmpProxy != null && !tmpProxy.equals(prx))
				result.put(sourceAddr, tmpProxy);
			tmpProxy = (Proxy) proxies.get(targetAddr);
			if (tmpProxy != null && !tmpProxy.equals(prx))
				result.put(targetAddr, tmpProxy);
		}

		return result;
	}
	
	public Proxy findPlaceProxy(Place p) {
		for (ObjectImpl oi : proxies.values()) {
			Proxy prx = (Proxy) oi;
			List<FeatureValue> fv = getFeatureValue(prx, "place_id");
			StringValue placeID = (StringValue) fv;
			if (Long.parseLong(placeID.val) == p.id) {
				return prx;
			}
		}
		return null;
	}

}
