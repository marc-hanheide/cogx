package motivation.util.facades;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import motivation.util.castextensions.WMEntrySet;
import Ice.ObjectImpl;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.PerceivedEntity;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import cast.architecture.ManagedComponent;

public class BinderFacade {
	WMEntrySet unionConfigurations;
	WMEntrySet proxies;
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
	}

	public void start() {
		unionConfigurations.start();
		proxies.start();
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
			component.log("  check feature: " + f.featlabel);
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
		component.log("unions:  " + unionConfigurations.size());
		component.log("proxies: " + proxies.size());
		for (Union union : unions) {
			component.log("check union " + union.entityID);
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

}
