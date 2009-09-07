package binder.fakeproxygenerators;

import binder.autogen.core.*;

public class FakeComsysProxyGenerator extends AbstractProxyGenerator {
	
	public FakeComsysProxyGenerator() {
		super();
	}
	
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
			return createProxyTwo();
		}
		return null;
	}
	

	private Proxy createProxyOne() {
		
		Proxy proxy = createNewProxy ("fakecomsys", 0.95f);
		
		FeatureValue mug = createStringValue("mug", 0.91f);
		Feature feat = createFeatureWithUniqueFeatureValue ("ling_label", mug);
		addFeatureToProxy (proxy, feat);
		
		return proxy;
	}
	
	
	private Proxy createProxyTwo() {
		Proxy proxy = createNewProxy ("fakecomsys", 0.85f);
	
		FeatureValue ball = createStringValue ("ball", 0.8f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("ling_label", ball);
		addFeatureToProxy (proxy, feat1);

		FeatureValue red = createStringValue ("red", 0.83f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("ling_colour", red);
		addFeatureToProxy (proxy, feat2);

		return proxy;
	}

}
