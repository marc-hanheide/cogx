package motivation.util.facades;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import motivation.util.castextensions.WMEntrySet;
import Ice.ObjectImpl;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.RelationProxy;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;

public class BinderFacade {
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

	public Map<WorkingMemoryAddress, RelationProxy> findRelationBySrc(
			String src) {
		Map<WorkingMemoryAddress, RelationProxy> result=  new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> e : relations.entrySet()) {
			RelationProxy rp = (RelationProxy) e.getValue();
			if (((AddressValue) rp.source.alternativeValues[0]).val.equals(src))
				result.put(e.getKey(), rp);
		}
		return result;

	}

	public Map<WorkingMemoryAddress, RelationProxy> findRelationBytarget(
			String target) {
		Map<WorkingMemoryAddress, RelationProxy> result=  new HashMap<WorkingMemoryAddress, RelationProxy>();
		for (Entry<WorkingMemoryAddress, ObjectImpl> e : relations.entrySet()) {
			RelationProxy rp = (RelationProxy) e.getValue();
			if (((AddressValue) rp.target.alternativeValues[0]).val.equals(target))
				result.put(e.getKey(), rp);
		}
		return result;

	}

}
