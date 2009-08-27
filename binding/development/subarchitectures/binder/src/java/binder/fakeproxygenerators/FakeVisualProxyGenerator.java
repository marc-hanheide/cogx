package binder.fakeproxygenerators;

import java.util.Enumeration;
import java.util.Random;
import java.util.Vector;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.core.*;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbDistribUtils;

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
	

	private Proxy createProxyOne() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakevision"; 
		proxy.probExists = 0.75f;
		
		proxy.features = new Feature[2];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "obj_label";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.8f,"mug");
		
		proxy.features[1] = new Feature();
		proxy.features[1].featlabel = "colour";
		proxy.features[1].alternativeValues = new FeatureValue[1];
		proxy.features[1].alternativeValues[0] = new StringValue(0.95f,"blue");
		
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);

		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);
		
		return proxy;
	}


	private Proxy createProxyTwo() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakevision"; 
		proxy.probExists = 0.9f;
	
		proxy.features = new Feature[3];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "obj_label";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.8f,"ball");
		
		proxy.features[1] = new Feature();
		proxy.features[1].featlabel = "colour";
		proxy.features[1].alternativeValues = new FeatureValue[2];
		proxy.features[1].alternativeValues[0] = new StringValue(0.65f,"red");
		proxy.features[1].alternativeValues[1] = new StringValue(0.2f,"blue");
		
		proxy.features[2] = new Feature();
		proxy.features[2].featlabel = "location";
		proxy.features[2].alternativeValues = new FeatureValue[1];
		proxy.features[2].alternativeValues[0] = new StringValue(0.5f,"on_table");
		
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);

		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);

		return proxy;
	}
	
	

}
