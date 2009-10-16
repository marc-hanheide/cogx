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
import binder.autogen.core.*;

/**
 * Fake proxy generator for vision
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 25/08/2009)
 */
public class FakeVisualProxyGenerator extends AbstractProxyGenerator {

	
	/**
	 * Start
	 */
	public void start () {
		log("Fake visual proxy generator successfully started");
	}
	
	/**
	 * Run
	 */
	public void run() {
		randomInsertion();
	}
	
	/**
	 * Create one index proxy
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
	 * Create proxy one (blue mug)
	 * 
	 * @return the proxy
	 */
	protected Proxy createProxyTwo() {
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakevision", "blublu", "VisualObject");
		Proxy proxy = createNewProxy (origin, 0.75f);
		
		FeatureValue mug = createStringValue ("mug", 0.8f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("obj_label", mug);
		addFeatureToProxy (proxy, feat1);
		
		FeatureValue blue = createStringValue ("blue", 0.95f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("colour", blue);
		addFeatureToProxy (proxy, feat2);
		
		FeatureValue highsaliency = createStringValue("high", 0.9f);
		Feature feat3 = createFeatureWithUniqueFeatureValue ("saliency", highsaliency);
		addFeatureToProxy (proxy, feat3);
		
		return proxy;
	}
  
 
	/**
	 * Create proxy two (red ball on a table)
	 * 
	 * @return the proxy
	 */
	protected Proxy createProxyOne() {
		
		sleepComponent(100);
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakevision", "blublu2", "VisualObject");
		Proxy proxy = createNewProxy (origin, 0.9f);
		
		FeatureValue ball = createStringValue ("ball", 0.81f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("obj_label", ball);
		addFeatureToProxy (proxy, feat1);		
	
	
		FeatureValue red = createStringValue("red", 0.75f);
		FeatureValue blue = createStringValue("blue", 0.2f);
		FeatureValue[] vals = {red, blue};
		Feature feat2 = createFeatureWithAlternativeFeatureValues ("colour", vals);
		addFeatureToProxy (proxy, feat2);
		

		FeatureValue ontable = createStringValue("on_table", 0.7f);
		Feature feat3 = createFeatureWithUniqueFeatureValue ("location", ontable);
		addFeatureToProxy (proxy, feat3);
		
		FeatureValue lowsaliency = createStringValue("low", 0.9f);
		Feature feat4 = createFeatureWithUniqueFeatureValue ("saliency", lowsaliency);
		addFeatureToProxy (proxy, feat4);
		
		return proxy;
	}
	
	

}
