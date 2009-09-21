package binder.tests;

import java.util.Map;
import java.util.Random;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.OriginInfo;
import binder.autogen.core.Proxy;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;


public class Test2 extends AbstractTester{

	static int testNumber = 2;
	static String task = "Generate correct single-proxy unions for a large set of random proxies";
	
	
	int NUMBER_OF_PROXIES = 30;
	int MAX_NUMBER_OF_FEATURES = 6;
	int MAX_NUMBER_OF_FEATUREVALUES = 4;
	int NUMBER_SUBARCHS = 4;
	
	UnionConfiguration curConfig;
	
	Class[] featValueTypes = {StringValue.class, IntegerValue.class, BooleanValue.class};

	
	int proxyCount = 1;
	int featurecount = 1;
	int featurevaluecount = 1;
	
	
	public Test2 () {
		super(testNumber, task);
	}
	
	
	
	@Override
	public void start() {

		// if best selected UnionConfiguration has been updated in the WM, update the
		// reader accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UnionConfiguration config = 
						getMemoryEntry(_wmc.address, UnionConfiguration.class);
					curConfig = config;
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
	}


	/**
	 * Set configuration parameters
	 */

	@Override
	public void configure(Map<String, String> _config) {
		
		if (_config.containsKey("--nbproxies")) {
			NUMBER_OF_PROXIES = Integer.parseInt(_config.get("--nbproxies"));
		} 
		if (_config.containsKey("--nbfeatures")) {
			MAX_NUMBER_OF_FEATURES = Integer.parseInt(_config.get("--nbfeatures"));
		}
		if (_config.containsKey("--nbfeatvals")) {
			MAX_NUMBER_OF_FEATUREVALUES = Integer.parseInt(_config.get("--nbfeatvals"));
		}
		if (_config.containsKey("--nbsubarchs")) {
			NUMBER_SUBARCHS = Integer.parseInt(_config.get("--nbsubarchs"));
		}		
	}
	
	
	@Override
	public boolean performTest() {
		
		for (int i = 0 ; i < NUMBER_OF_PROXIES; i++) {
			log("Now creating random proxy " + (i + 1));
			Proxy proxy = createNewRandomProxy();
			addProxyToWM(proxy);
			sleepComponent(20);
		}
		
		int counts = 0;
		while ((curConfig.includedUnions.length < NUMBER_OF_PROXIES) && counts < (NUMBER_OF_PROXIES*2)) {
		try {
			sleepComponent(150);
			counts++;
		}
		catch (Exception e){ }
		}
		log("Final number of unions: " + curConfig.includedUnions.length);
		
		boolean result = (curConfig.includedUnions.length == NUMBER_OF_PROXIES);
		
		if (!result) {
			System.out.println("Final number of unions: " + curConfig.includedUnions.length);
		}
		
		return result;
	}
	
	
	public Proxy createNewRandomProxy() {
		Random rand = new Random();
		int subarchNb = rand.nextInt(NUMBER_SUBARCHS) + 1;
		OriginInfo origin = createOriginInfo ("subarch"+subarchNb, "localID"+proxyCount, "localType");
		proxyCount++;
		float probExists = rand.nextFloat();
		Proxy proxy = createNewProxy(origin, probExists);
		
		int nbFeatures = rand.nextInt(MAX_NUMBER_OF_FEATURES) ;
		for (int i = 0 ; i < nbFeatures; i++) {
			Feature feat = createFeature("featlabel"  + featurecount);
			featurecount++;
			int nbFeatureValues = rand.nextInt(MAX_NUMBER_OF_FEATUREVALUES) ;
			
			for (int j = 0; j < nbFeatureValues ;j++) {
				FeatureValue featvalue = createRandomFeatureValue();
				addFeatureValueToFeature(feat, featvalue);
			}
			if (feat.alternativeValues.length > 0) {
				addFeatureToProxy(proxy, feat);
			}
		}
		
		return proxy;
	}

	
	public FeatureValue createRandomFeatureValue() {

		featurevaluecount++;
		Random random = new Random();
		int featvalueTypeChooser = random.nextInt(featValueTypes.length);
		Class featureValueType = featValueTypes[featvalueTypeChooser];
		
		if (featureValueType.equals(StringValue.class)) {
			return createStringValue("str"+ featurevaluecount , 
					random.nextFloat()/((float)MAX_NUMBER_OF_FEATUREVALUES));
		}
		else if (featureValueType.equals(IntegerValue.class)) {
			return createIntegerValue(featurevaluecount, 
					random.nextFloat()/((float)MAX_NUMBER_OF_FEATUREVALUES));
		}
		else if (featureValueType.equals(BooleanValue.class)) {
			return createBooleanValue(random.nextBoolean(), 
					random.nextFloat()/((float)MAX_NUMBER_OF_FEATUREVALUES));
		}
			
		return null;
		}
	
}
