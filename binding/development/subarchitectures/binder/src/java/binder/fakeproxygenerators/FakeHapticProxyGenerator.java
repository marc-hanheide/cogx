package binder.fakeproxygenerators;

import binder.autogen.core.*;
import binder.autogen.featvalues.StringValue;

public class FakeHapticProxyGenerator extends AbstractProxyGenerator {

	String proxyOneId;
	String proxyTwoId;
	
	public void start () {
		log("Fake haptic proxy generator successfully started");
	}
	
	
	public void run() {
		randomInsertion();
	}
	
	
	public Proxy createProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			return createProxyTwo();
		}
		
		if (nb == 3) {
			return createRelationProxy();
		}
		return null;
	}
	

	private Proxy createProxyOne() {
		
		Proxy proxy = createNewProxy("fakehaptic", 0.35f);
		
		FeatureValue cylindrical = createStringValue ("cylindrical", 0.73f);
		Feature feat = createFeatureWithUniqueFeatureValue ("shape", cylindrical);
		addFeatureToProxy (proxy, feat);
		
		proxyOneId = proxy.entityID;
		
		return proxy;
	}
	
	

	private Proxy createProxyTwo() {
		Proxy proxy = createNewProxy ("fakehaptic", 0.75f);
		
		FeatureValue spherical = createStringValue ("spherical", 0.67f);
		Feature feat1 = createFeatureWithUniqueFeatureValue("shape", spherical);
		addFeatureToProxy (proxy, feat1);
	
		FeatureValue trueval = createStringValue ("true", 0.8f);
		FeatureValue falseval = createStringValue ("false", 0.15f);
		FeatureValue[] vals = {trueval, falseval};
		Feature feat2 = createFeatureWithAlternativeFeatureValues("graspable", vals);
		addFeatureToProxy (proxy, feat2);
		
		proxyTwoId = proxy.entityID;
		
		return proxy;
	}
	
	
	private Proxy createRelationProxy() {
		
		StringValue[] sources = new StringValue[1];
		sources[0] = createStringValue(proxyOneId, 0.9f);
		
		StringValue[] targets = new StringValue[1];
		targets[0] = createStringValue(proxyTwoId, 0.91f);
		
		Proxy proxy = createNewRelationProxy("fakehaptic", 0.81f, sources, targets);
		
		return proxy;
	}
	
}
