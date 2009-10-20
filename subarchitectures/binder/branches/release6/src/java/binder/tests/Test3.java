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
import java.util.Vector;

import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.WorkingMemoryPointer;

import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;

/**
 * Test 3: generate correct single and multi-proxy unions for random proxies and an associated
 * bayesian network
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 23/09/2009)
 */
public class Test3 extends AbstractTester{

	static int testNumber = 3;
	static String task = "Generate correct single- and multi-proxy unions for random proxies";
	
		
		// number of proxies
		int NUMBER_OF_PROXIES = 30;
		
		int MIN_NUMBER_OF_FEATURES = 1;
		// max number of features per proxy
		int MAX_NUMBER_OF_FEATURES = 6;
		
		
		int MIN_NUMBER_OF_FEATUREVALUES = 1;
		// max number of feature values per feature
		int MAX_NUMBER_OF_FEATUREVALUES = 1;
		
		// number of subarchitectures
		int NUMBER_SUBARCHS = 4;
		
		
		int nbProxiesWithFeat1FeatVal1 = 0;
		int nbProxiesWithFeat2FeatVal1 = 0;
		int nbProxiesWithFeat1FeatVal2 = 0;
		int nbProxiesWithFeat2FeatVal2 = 0;
		
		
		// current union configuration
		UnionConfiguration curConfig;
		
		// various counters
		int proxyCount = 1;
		
		/**
		 * Initialise
		 */
		public Test3 () {
			super(testNumber, task);
		}
		
		
		/**
		 * Configure (add a change filter on union configurations)
		 */
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
		
		/**
		 * Perform the test
		 * 
		 * @return true if test successful, false otherwise
		 */
		@Override
		public boolean performTest() {
			
			for (int i = 0 ; i < NUMBER_OF_PROXIES; i++) {
				log("Now creating random proxy " + (i + 1));
			
				Proxy proxy = null;
		/**	if (i == 0)
					proxy = createProxyOne();
				else if (i == 1)
					proxy = createProxyTwo();  */
				
				proxy = createNewRandomProxy();
				
				addProxyToWM(proxy);
				sleepComponent(20);
			}
			
			
			int expectedNumberOfDoubleProxyUnions = 
				Math.min(nbProxiesWithFeat1FeatVal1, nbProxiesWithFeat2FeatVal1) + 
				Math.min(nbProxiesWithFeat1FeatVal2, nbProxiesWithFeat2FeatVal2);
			
			int counts = 0;
			while (getNumberOfDoubleProxyUnions(curConfig) != expectedNumberOfDoubleProxyUnions &&
					counts < (NUMBER_OF_PROXIES*2)) {
			try {
				sleepComponent(60);
				counts++;
			}
			catch (Exception e){ }
			}
			
			log("Final number of unions: " + curConfig.includedUnions.length);
			
			log("Number of double proxy unions: " + getNumberOfDoubleProxyUnions(curConfig));	
			log("Expected number of double proxy unions: "  + expectedNumberOfDoubleProxyUnions);
			
			
			boolean result = (getNumberOfDoubleProxyUnions(curConfig) == expectedNumberOfDoubleProxyUnions);
						
			if (!result) {
				log("Number of double proxy unions: " + getNumberOfDoubleProxyUnions(curConfig));	
				log("Expected number of double proxy unions: "  + expectedNumberOfDoubleProxyUnions);
			}
			
			return result;
		}
		
		
		private int getNumberOfDoubleProxyUnions (UnionConfiguration config) {
			
			int counter = 0;
			for (int i = 0 ; i < config.includedUnions.length ; i++) {
				Union u = config.includedUnions[i];
				if (u.includedProxies.length == 2) {
					counter++;
				}
			}
			return counter;
		}
	
		
		
		/**
		 * Created a random proxy
		 * 
		 * @return the new proxy
		 */
		public Proxy createNewRandomProxy() {
			Random rand = new Random();
			int subarchNb = rand.nextInt(NUMBER_SUBARCHS) + 1;
			WorkingMemoryPointer origin = createWorkingMemoryPointer("subarch"+subarchNb, "localID"+proxyCount, "localType");
			proxyCount++;
			float probExists = 0.5f + (rand.nextFloat()/2.0f);
			
			Proxy proxy = createNewProxy(origin, probExists);
			
			int nbFeatures = rand.nextInt(MAX_NUMBER_OF_FEATURES+1) ;
		
			if (nbFeatures < MIN_NUMBER_OF_FEATURES) {
				nbFeatures = MIN_NUMBER_OF_FEATURES;
			}
		
			Vector<Integer> alreadyUsedTypes = new Vector<Integer>();
			for (int i = 0 ; i < nbFeatures; i++) {

				int featType = rand.nextInt(MAX_NUMBER_OF_FEATURES) + 1;
				while (alreadyUsedTypes.contains(featType)) {
					featType = rand.nextInt(MAX_NUMBER_OF_FEATURES) + 1;
				}
				Feature feat = createFeature("feat"  + featType + "_sub" + subarchNb);
				alreadyUsedTypes.add(featType);
				
				int nbFeatureValues = rand.nextInt(MAX_NUMBER_OF_FEATUREVALUES+1) ;
			
				if (nbFeatureValues < MIN_NUMBER_OF_FEATUREVALUES) {
					nbFeatureValues = MIN_NUMBER_OF_FEATUREVALUES;
				}
				for (int j = 0; j < nbFeatureValues ;j++) {
					FeatureValue featvalue = createRandomFeatureValue();
					addFeatureValueToFeature(feat, featvalue);
					
				}
				if (feat.alternativeValues.length > 0) {
					addFeatureToProxy(proxy, feat);
					
				}
			}
			
			incrementCounters(proxy);
			
			return proxy;
		} 

		
		private boolean countainsFeatureValuePair (Proxy proxy, String feat, String val) {
			
			for (int i = 0 ; i < proxy.features.length ; i++) {
				Feature f = proxy.features[i];		
				if (f.featlabel.equals (feat)) {				
					for (int j = 0 ; j < f.alternativeValues.length ; j++) {
						if (((StringValue)f.alternativeValues[j]).val.equals(val)) {
							return true;
						}
					}
				}
			}
			return false;
		}
		
		private void incrementCounters (Proxy proxy) {
			
			if (countainsFeatureValuePair(proxy, "feat1_sub1", "featval_1") &&
					!countainsFeatureValuePair(proxy, "feat1_sub1", "featval_2") &&
					!countainsFeatureValuePair(proxy, "feat2_sub1", "featval_1")) {
				nbProxiesWithFeat1FeatVal1++;
			}
			else if (countainsFeatureValuePair(proxy, "feat1_sub1", "featval_2") &&
					!countainsFeatureValuePair(proxy, "feat1_sub1", "featval_1")) {
				nbProxiesWithFeat1FeatVal2++;
			}
			else if (countainsFeatureValuePair(proxy, "feat1_sub2", "featval_1") &&
					!countainsFeatureValuePair(proxy, "feat1_sub2", "featval_2")) {
				nbProxiesWithFeat2FeatVal1++;
			}
			else if (countainsFeatureValuePair(proxy, "feat1_sub2", "featval_2") &&
					!countainsFeatureValuePair(proxy, "feat1_sub2", "featval_1")) {
				nbProxiesWithFeat2FeatVal2++;
			}
		}
		
		
		/**
		 * Create a random feature value
		 * 
		 * @return the new feature value
		 */
		public FeatureValue createRandomFeatureValue() {

			Random random = new Random();
			int featvalueType = random.nextInt(MAX_NUMBER_OF_FEATUREVALUES) + 1;
			
			return createStringValue("featval_"+ featvalueType, random.nextFloat());
		}
	}
