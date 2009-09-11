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

public class FakeComsysProxyGenerator extends AbstractProxyGenerator {
	
	Proxy proxyOne;
	
	public FakeComsysProxyGenerator() {
		super();
	}
	
	public void start () {
		log("Fake visual proxy generator successfully started");
	}

	
	public void run() {
		randomInsertion();
	}
	
	
	public Proxy createProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			deleteEntityInWM(proxyOne);
			return createProxyTwo();
		}
		return null;
	}
	

	private Proxy createProxyOne() {
		
		OriginInfo origin = createOriginInfo ("fakecomsys", "blabla", "Referent");
		Proxy proxy = createNewProxy (origin, 0.95f);
		
		FeatureValue mug = createStringValue("mug", 0.91f);
		Feature feat = createFeatureWithUniqueFeatureValue ("ling_label", mug);
		addFeatureToProxy (proxy, feat);
		proxyOne = proxy;
		
		return proxy;
	}
	
	
	private Proxy createProxyTwo() {
		
		OriginInfo origin = createOriginInfo ("fakecomsys", "blabla2", "Referent");
		Proxy proxy = createNewPhantomProxy (origin, 0.85f);
	
		FeatureValue ball = createStringValue ("ball", 0.8f);
		Feature feat1 = createFeatureWithUniqueFeatureValue ("ling_label", ball);
		addFeatureToProxy (proxy, feat1);

		FeatureValue red = createStringValue ("red", 0.83f);
		Feature feat2 = createFeatureWithUniqueFeatureValue ("ling_colour", red);
		addFeatureToProxy (proxy, feat2);

		return proxy;
	}

}
