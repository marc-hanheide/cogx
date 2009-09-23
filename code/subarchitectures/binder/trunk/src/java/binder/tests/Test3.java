// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
//                                                                                                                          
// This library is free software; you can redistribute it and/or                                                            
// modify it under the terms of the GNU Lesser General Public License                                                       
// as published by the Free Software Foundation; either version 2.1 of                                                      
// the License, or (at your option) any later version.                                                                      
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but                                                      
// WITHOUT ANY WARRANTY; without even the implied warranty of                                                               
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU                                                         
// Lesser General Public License for more details.                                                                          
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public                                                         
// License along with this program; if not, write to the Free Software                                                      
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA                                                                
// 02111-1307, USA.                                                                                                         
// =================================================================                                                        

package binder.tests;

import java.util.Map;
import java.util.Random;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;

/**
 * Test 3: (to be filled)
 * 
 * @author Pierre Lison
 * @version 23/09/2009
 * @started 23/09/2009
 */
public class Test3 extends AbstractTester{

	static int testNumber = 2;
	static String task = "Generate correct single- and multi-proxy unions for random proxies";
	
	
	int NUMBER_OF_PROXIES = 30;
	int MAX_NUMBER_OF_FEATURES = 6;
	int MAX_NUMBER_OF_FEATUREVALUES = 4;
	int NUMBER_SUBARCHS = 4;
	
	UnionConfiguration curConfig;
	
	Class[] featValueTypes = {StringValue.class, IntegerValue.class, BooleanValue.class};

	
	int proxyCount = 1;
	int featurecount = 1;
	int featurevaluecount = 1;
	
	
	public Test3 () {
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
		
			
		return true;
	}
	
	
	public Proxy createNewRandomProxy() {
		Random rand = new Random();
		int subarchNb = rand.nextInt(NUMBER_SUBARCHS) + 1;
		WorkingMemoryPointer origin = createWorkingMemoryPointer("subarch"+subarchNb, "localID"+proxyCount, "localType");
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
