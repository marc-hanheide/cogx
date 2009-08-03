package binder.fakeproxygenerators;

import java.util.Enumeration;
import java.util.Random;
import java.util.Vector;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.core.*;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbDistribUtils;

public class FakeComsysProxyGenerator extends AbstractProxyGenerator {

	public void start () {
		log("Fake visual proxy generator successfully started");
	}
	
	public void run() {
		
		sleepComponent(1000);

		Proxy p1 = createProxyOne();
		addEntityToWM(p1);
		
		sleepComponent(1500);

		Proxy p2 = createProxyTwo();
		addEntityToWM(p2);
	}
	

	private Proxy createProxyOne() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakecomsys"; 
		proxy.probExists = 0.95f;
	
		proxy.features = new Feature[1];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "ling_label";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.8f,"mug");
	
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);

		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);

		return proxy;
	}
	
	
	private Proxy createProxyTwo() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakecomsys"; 
		proxy.probExists = 0.85f;
	
		proxy.features = new Feature[2];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "ling_label";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.8f,"ball");

		proxy.features[1] = new Feature();
		proxy.features[1].featlabel = "ling_colour";
		proxy.features[1].alternativeValues = new FeatureValue[1];
		proxy.features[1].alternativeValues[0] = new StringValue(0.8f,"red");
		
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);

		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);

		return proxy;
	}

}
