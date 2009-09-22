package binder.fakeproxygenerators;

import cast.cdl.WorkingMemoryPointer;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;


public class FakeProxyGeneratorForBeliefModel extends AbstractProxyGenerator {

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
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakevision", "blibli", "VisualObject");
		Proxy proxy = createNewProxy(origin, 0.35f);
		
		FeatureValue red = createStringValue ("red", 0.73f);
		Feature feat = createFeatureWithUniqueFeatureValue ("colour", red);
		addFeatureToProxy (proxy, feat);
				
		FeatureValue cylindrical = createStringValue ("cylindrical", 0.63f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("shape", cylindrical);
		addFeatureToProxy (proxy, feat2);
		
		log("Proxy one for belief model successfully created");
		return proxy;
	}
	


	private Proxy createProxyTwo() {
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakevision", "blibli2", "VisualObject");
		Proxy proxy = createNewProxy(origin, 0.35f);
		
		FeatureValue blue = createStringValue ("blue", 0.73f);
		FeatureValue red = createStringValue ("red", 0.23f);
		Feature feat = createFeature ("colour");
		addFeatureValueToFeature(feat, blue);
		addFeatureValueToFeature(feat,red);
		addFeatureToProxy (proxy, feat);
				
		FeatureValue spherical = createStringValue ("spherical", 0.63f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("shape", spherical);
		addFeatureToProxy (proxy, feat2);
		
		log("Proxy two for belief model successfully created");
		return proxy;
	}
	
}
