package binder.fakeproxygenerators;

import java.util.Enumeration;
import java.util.Random;
import java.util.Vector;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.binderEssentials.DiscreteFeatureValueDistribution;
import binder.autogen.binderEssentials.DiscreteFeatureValueWithProb;
import binder.autogen.binderEssentials.Feature;
import binder.autogen.binderEssentials.FeatureDistribution;
import binder.autogen.binderEssentials.FeatureValue;
import binder.autogen.binderEssentials.FeatureValueDistribution;
import binder.autogen.binderEssentials.Union;
import binder.autogen.binderEssentials.PerceivedEntity;
import binder.autogen.binderEssentials.Proxy;
import binder.autogen.binderEssentials.StringValue;

public class FakeVisualProxyGenerator extends AbstractProxyGenerator {

	Random randomProbGenerator;
	
	public void start () {
		randomProbGenerator = new Random();
		log("Fake visual proxy generator successfully started");
	}
	
	public void run() {
		
	//	Vector<Proxy> proxies = new Vector<Proxy>();
		sleepComponent(1500);
	/**	for (int i = 0; i < 1; i++) {
			sleepComponent(500);
			Proxy p = createNewRandomProxy();
			proxies.add(p);
			addEntityToWM(p);
			sleepComponent(500);
		}*/
		
		Proxy p1 = createProxyOne();
		addEntityToWM(p1);
		
	//	Union u = createNewRandomUnion(proxies);
	//	addEntityToWM(u);
	}
	
	
	

	private Proxy createProxyOne() {
		Proxy proxyOne = new Proxy();
		proxyOne.entityID = newDataID();
		proxyOne.subarchId = "fakevision";
		proxyOne.probExists = 0.9f;
		
		proxyOne.features = new Feature[1];
		proxyOne.features[0] = new Feature();
		proxyOne.features[0].featlabel = "obj_label";
		proxyOne.features[0].alternativeValues = new FeatureValue[1];
		proxyOne.features[0].alternativeValues[0] = new StringValue("mug");
		
		return proxyOne;
	}
	

	private Proxy createNewRandomProxy() {
		Proxy newProxy = new Proxy();
		newProxy.entityID = newDataID();
		newProxy.subarchId = "fakevision";
		newProxy.probExists = randomProbGenerator.nextFloat();

		newProxy.features = createNewRandomFeatures();

		return newProxy;
	}

	
	private Feature[] createNewRandomFeatures() {
		int nbFeats = randomProbGenerator.nextInt(4);

		Vector<Feature> features = new Vector<Feature>();
		
		for (int i = 0 ; i < nbFeats ; i++) {
			Feature feat = new Feature();
			feat.featlabel = ("Feature "+ i);

			int nbAlternatives = randomProbGenerator.nextInt(4);
			feat.alternativeValues = new FeatureValue[nbAlternatives];
			
			for (int j = 0; j < nbAlternatives; j++) {
				feat.alternativeValues[j] = new StringValue("value"+j);
			}
			
			features.add(feat);
		
		}
		
		Feature[] featuresArray = new Feature[features.size()];
		featuresArray = features.toArray(featuresArray);
		
		return featuresArray;
	}
	
	private void addEntityToWM(PerceivedEntity entity) {

		try {
		addToWorkingMemory(entity.entityID, entity);
		log("new Proxy succesfully added to the binding working memory");
		
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	private Union createNewRandomUnion(Vector<Proxy> proxies) {
		
		Union union = new Union();
		union.entityID = newDataID();
		union.probExists = randomProbGenerator.nextFloat();
		union.features = createNewRandomFeatures();
		
		union.includedProxies = new Proxy[proxies.size()];
		int z = 0;
		for (Enumeration<Proxy> e = proxies.elements(); e.hasMoreElements(); ) {
			union.includedProxies[z] = e.nextElement();
			z++;
		}
		
		return union;
	}

}
