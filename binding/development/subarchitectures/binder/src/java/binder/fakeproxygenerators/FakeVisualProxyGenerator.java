package binder.fakeproxygenerators;

import java.util.Enumeration;
import java.util.Random;
import java.util.Vector;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.core.*;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbabilityDistributionUtils;

public class FakeVisualProxyGenerator extends AbstractProxyGenerator {

	public void start () {
		log("Fake visual proxy generator successfully started");
	}
	
	public void run() {
		
		sleepComponent(500);

		Proxy p1 = createProxyOne();
		addEntityToWM(p1);
		
	}
	
	

	private Proxy createProxyOne() {
		Proxy proxyOne = new Proxy();
		proxyOne.entityID = newDataID();
		proxyOne.subarchId = "fakevision"; 
		proxyOne.probExists = 0.9f;
		
		proxyOne.features = new Feature[2];
		proxyOne.features[0] = new Feature();
		proxyOne.features[0].featlabel = "obj_label";
		proxyOne.features[0].alternativeValues = new FeatureValue[1];
		proxyOne.features[0].alternativeValues[0] = new StringValue(0.8f,"mug");
		
		proxyOne.features[1] = new Feature();
		proxyOne.features[1].featlabel = "colour";
		proxyOne.features[1].alternativeValues = new FeatureValue[1];
		proxyOne.features[1].alternativeValues[0] = new StringValue(0.6f,"blue");
		
		proxyOne.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(proxyOne);

		return proxyOne;
	}
	
	

}
