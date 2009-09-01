package binder.fakeproxygenerators;

import binder.autogen.core.*;

public class FakeVisualProxyGenerator extends AbstractProxyGenerator {

	public void start () {
		log("Fake visual proxy generator successfully started");
	}
	
	
	public void run() {
		randomInsertion();
	}
	
	public Proxy createProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			sleepComponent(2000);
			return createProxyTwo();
		}
		return null;
	}
	

	protected Proxy createProxyOne() {
		Proxy proxy = createNewProxy ("fakevision", 0.75f);
		
		FeatureValue mug = createStringValue ("mug", 0.8f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("obj_label", mug);
		addFeatureToProxy (proxy, feat1);
		
		FeatureValue blue = createStringValue ("blue", 0.95f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("colour", blue);
		addFeatureToProxy (proxy, feat2);
		
		return proxy;
	}


	protected Proxy createProxyTwo() {
		Proxy proxy = createNewProxy ("fakevision", 0.9f);
		
		FeatureValue ball = createStringValue ("ball", 0.81f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("obj_label", ball);
		addFeatureToProxy (proxy, feat1);		
	
	
		FeatureValue red = createStringValue("red", 0.65f);
		FeatureValue blue = createStringValue("blue", 0.2f);
		FeatureValue[] vals = {red, blue};
		Feature feat2 = createFeatureWithAlternativeFeatureValues ("colour", vals);
		addFeatureToProxy (proxy, feat2);
		

		FeatureValue ontable = createStringValue("on_table", 0.5f);
		Feature feat3 = createFeatureWithUniqueFeatureValue ("location", ontable);
		addFeatureToProxy (proxy, feat3);
		
		return proxy;
	}
	
	

}
