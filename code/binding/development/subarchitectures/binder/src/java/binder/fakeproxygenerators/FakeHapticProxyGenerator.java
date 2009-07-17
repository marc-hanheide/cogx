package binder.fakeproxygenerators;

import java.util.Enumeration;
import java.util.Random;
import java.util.Vector;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.core.*;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbabilityDistributionUtils;

public class FakeHapticProxyGenerator extends AbstractProxyGenerator {

	
	public void start () {
		log("Fake haptic proxy generator successfully started");
	}
	
	public void run() {
		
		sleepComponent(1000);
		
		Proxy p1 = createProxyOne();
		addEntityToWM(p1);
		
		sleepComponent(1200);
		
		Proxy p2 = createProxyTwo();
		addEntityToWM(p2);		
	}


	private Proxy createProxyOne() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakehaptic";
		proxy.probExists = 0.75f;
	
		proxy.features = new Feature[1];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "shape";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.73f, "cylindrical");
		
		proxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(proxy);
		return proxy;
	}
	
	

	private Proxy createProxyTwo() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakehaptic";
		proxy.probExists = 0.75f;
	
		proxy.features = new Feature[2];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "shape";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.67f, "spherical");
		
		proxy.features[1] = new Feature();
		proxy.features[1].featlabel = "graspable";
		proxy.features[1].alternativeValues = new FeatureValue[2];	
		proxy.features[1].alternativeValues[0] = new StringValue(0.8f, "true");
		proxy.features[1].alternativeValues[1] = new StringValue(0.15f, "false");
		
		proxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(proxy);
		return proxy;
	}
	
}
