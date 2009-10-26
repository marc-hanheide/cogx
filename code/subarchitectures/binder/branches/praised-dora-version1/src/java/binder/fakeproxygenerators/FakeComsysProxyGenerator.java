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
import binder.autogen.specialentities.PhantomProxy;
import binder.constructors.ProxyConstructor;

 
/**
 * Fake proxy generator for ComSys: creation of fake phantom proxies, insertion into the
 * binder working memory, and prediction of best binding unions for these phantom
 * proxies
 * 
 * @author Pierre Lison
 * @version 22/09/2009 (started 05/09/2009)
 * 
 */
public class FakeComsysProxyGenerator extends AbstractPhantomProxyGenerator {

	/**
	 * Start
	 */
	public void start () {
		log("Fake comsys proxy generator successfully started");
	}

	/**
	 * Run
	 */
	public void run() {
		randomPrediction();
	}
	
	/**
	 * Create one indexed phantom proxy
	 */
	public PhantomProxy createPhantomProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			return createProxyTwo();
		}
		return null;
	}
	
	/**
	 * Create proxy one ("mug")
	 * 
	 * @return the proxy
	 */
	private PhantomProxy createProxyTwo() {
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakecomsys", "blabla", "Referent");
		PhantomProxy proxy = ProxyConstructor.createNewPhantomProxy (origin, newDataID(), 0.95f);
		
		FeatureValue mug = createStringValue("mug", 0.91f);
		Feature feat = createFeatureWithUniqueFeatureValue ("ling_label", mug);
		addFeatureToProxy (proxy, feat);
		
		return proxy;
	}
	
	/**
	 * Create proxy two ("red ball")
	 * 
	 * @return the proxy
	 */
	private PhantomProxy createProxyOne() {
		
		WorkingMemoryPointer origin = createWorkingMemoryPointer ("fakecomsys", "blabla2", "Referent");
		PhantomProxy proxy = ProxyConstructor.createNewPhantomProxy (origin, newDataID(), 0.85f);
	
		FeatureValue ball = createStringValue ("ball", 0.8f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("ling_label", ball);
		addFeatureToProxy (proxy, feat1);

		FeatureValue red = createStringValue ("red", 0.83f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("ling_attribute", red);
		addFeatureToProxy (proxy, feat2);

		return proxy;
	}

}
