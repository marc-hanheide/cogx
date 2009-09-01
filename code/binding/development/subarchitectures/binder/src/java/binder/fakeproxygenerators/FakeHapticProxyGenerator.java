package binder.fakeproxygenerators;

import binder.autogen.core.*;

public class FakeHapticProxyGenerator extends AbstractProxyGenerator {

	
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
		return null;
	}
	


	private Proxy createProxyOne() {
		
		Proxy proxy = createNewProxy("fakehaptic", 0.35f);
		
		FeatureValue cylindrical = createStringValue ("cylindrical", 0.73f);
		Feature feat = createFeatureWithUniqueFeatureValue ("shape", cylindrical);
		addFeatureToProxy (proxy, feat);
		
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
		
		return proxy;
	}
	
}
