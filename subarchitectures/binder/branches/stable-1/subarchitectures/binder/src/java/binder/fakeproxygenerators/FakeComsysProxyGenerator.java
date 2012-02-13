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

import binder.autogen.core.*;
import binder.autogen.specialentities.PhantomProxy;

public class FakeComsysProxyGenerator extends AbstractPhantomProxyGenerator {

	
	public FakeComsysProxyGenerator() {
		super();
	}
	
	public void start () {
		log("Fake comsys proxy generator successfully started");
	}

	
	public void run() {
		randomPrediction();
	}
	
	
	public PhantomProxy createPhantomProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			return createProxyTwo();
		}
		return null;
	}
	
	
	private PhantomProxy createProxyOne() {
		
		OriginInfo origin = createOriginInfo ("fakecomsys", "blabla", "Referent");
		PhantomProxy proxy = createNewPhantomProxy (origin, 0.95f);
		
		FeatureValue ball = createStringValue("ball", 0.91f);
		Feature feat = createFeatureWithUniqueFeatureValue ("ling_label", ball);
		addFeatureToProxy (proxy, feat);
		
		return proxy;
	}
	
	
	private PhantomProxy createProxyTwo() {
		
		OriginInfo origin = createOriginInfo ("fakecomsys", "blabla2", "Referent");
		PhantomProxy proxy = createNewPhantomProxy (origin, 0.85f);
	
		FeatureValue ball = createStringValue ("ball", 0.8f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("ling_label", ball);
		addFeatureToProxy (proxy, feat1);

		FeatureValue red = createStringValue ("red", 0.83f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("ling_colour", red);
		addFeatureToProxy (proxy, feat2);

		return proxy;
	}

}