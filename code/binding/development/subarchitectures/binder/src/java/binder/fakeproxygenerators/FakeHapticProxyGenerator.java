package binder.fakeproxygenerators;

import java.util.Enumeration;
import java.util.Random;
import java.util.Vector;

import binder.abstr.AbstractProxyGenerator;
import binder.autogen.core.*;
import binder.autogen.featvalues.StringValue;
import binder.utils.ProbDistribUtils;

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
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakehaptic";
		proxy.probExists = 0.35f;
	
		proxy.features = new Feature[1];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "shape";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.73f, "cylindrical");
		
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);
		
		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);
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
		
		proxy.features = ProbDistribUtils.addIndeterminateFeatureValues(proxy.features);

		proxy.distribution = ProbDistribUtils.generateProbabilityDistribution(proxy);
		return proxy;
	}
	
}
