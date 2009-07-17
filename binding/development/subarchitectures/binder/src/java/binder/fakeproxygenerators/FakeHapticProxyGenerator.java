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
		
		sleepComponent(1500);
		
		Proxy p1 = createProxyOne();
		addEntityToWM(p1);
		
	}


	private Proxy createProxyOne() {
		Proxy proxyOne = new Proxy();
		proxyOne.entityID = newDataID();
		proxyOne.subarchId = "fakehaptic";
		proxyOne.probExists = 0.75f;
	
		proxyOne.features = new Feature[1];
		proxyOne.features[0] = new Feature();
		proxyOne.features[0].featlabel = "shape";
		proxyOne.features[0].alternativeValues = new FeatureValue[1];
		proxyOne.features[0].alternativeValues[0] = new StringValue(0.73f, "cylindrical");
		proxyOne.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(proxyOne);
		return proxyOne;
	}
	
}
