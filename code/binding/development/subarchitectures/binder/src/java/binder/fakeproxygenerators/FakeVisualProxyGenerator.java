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
		
		sleepComponent(1000);

		Proxy p2 = createProxyTwo();
		addEntityToWM(p2);
	}
	

	private Proxy createProxyOne() {
		Proxy proxy = new Proxy();
		proxy.entityID = newDataID();
		proxy.subarchId = "fakevision"; 
		proxy.probExists = 0.9f;
		
		proxy.features = new Feature[2];
		proxy.features[0] = new Feature();
		proxy.features[0].featlabel = "obj_label";
		proxy.features[0].alternativeValues = new FeatureValue[1];
		proxy.features[0].alternativeValues[0] = new StringValue(0.8f,"mug");
		
		proxy.features[1] = new Feature();
		proxy.features[1].featlabel = "colour";
		proxy.features[1].alternativeValues = new FeatureValue[1];
		proxy.features[1].alternativeValues[0] = new StringValue(0.6f,"blue");
		
		proxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(proxy);

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
		proxy.features[1].alternativeValues[0] = new StringValue(0.6f,"red");
		proxy.features[1].alternativeValues[1] = new StringValue(0.6f,"pink");
		
		proxy.features[2] = new Feature();
		proxy.features[2].featlabel = "location";
		proxy.features[2].alternativeValues = new FeatureValue[1];
		proxy.features[2].alternativeValues[0] = new StringValue(0.5f,"on_table");
		
		proxy.distribution = ProbabilityDistributionUtils.generateProbabilityDistribution(proxy);

		return proxy;
	}
	
	

}
