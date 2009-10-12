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


package binder.fakeproxygenerators;

import cast.cdl.WorkingMemoryPointer;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.Proxy;

/**
 * Fake proxy generator used to test the belief model
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 18/09/2009)
 */
public class FakeProxyGeneratorForBeliefModel extends AbstractProxyGenerator {

	/**
	 * Start
	 */
	public void start () {
		log("Fake haptic proxy generator successfully started");
	}
	
	
	/**
	 * Run
	 */
	public void run() {
		randomInsertion();
	}
	
	/**
	 * Create one indexed proxy
	 */
	public Proxy createProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			return createProxyTwo();
		}
		return null;
	}
	
	/**
	 * Create proxy one (cylindrical, red object)
	 * 
	 * @return the proxy
	 */
	private Proxy createProxyOne() {
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakevision", "blibli", "VisualObject");
		Proxy proxy = createNewProxy(origin, 0.35f);
		
		FeatureValue red = createStringValue ("red", 0.73f);
		Feature feat = createFeatureWithUniqueFeatureValue ("colour", red);
		addFeatureToProxy (proxy, feat);
				
		FeatureValue cylindrical = createStringValue ("cylindrical", 0.63f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("shape", cylindrical);
		addFeatureToProxy (proxy, feat2);
		
		log("Proxy one for belief model successfully created");
		return proxy;
	}
	

	/**
	 * Create proxy two (spherical, blue/red object)
	 * 
	 * @return the proxy
	 */
	private Proxy createProxyTwo() {
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakevision", "blibli2", "VisualObject");
		Proxy proxy = createNewProxy(origin, 0.35f);
		
		FeatureValue blue = createStringValue ("blue", 0.73f);
		FeatureValue red = createStringValue ("red", 0.23f);
		Feature feat = createFeature ("colour");
		addFeatureValueToFeature(feat, blue);
		addFeatureValueToFeature(feat,red);
		addFeatureToProxy (proxy, feat);
				
		FeatureValue spherical = createStringValue ("spherical", 0.63f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("shape", spherical);
		addFeatureToProxy (proxy, feat2);
		
		log("Proxy two for belief model successfully created");
		return proxy;
	}
	
}
