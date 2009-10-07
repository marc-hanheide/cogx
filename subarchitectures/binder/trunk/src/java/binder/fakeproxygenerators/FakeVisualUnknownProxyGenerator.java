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
import binder.autogen.featvalues.*;

public class FakeVisualUnknownProxyGenerator extends AbstractProxyGenerator {

	public void start () {
		log("Fake visual proxy generator successfully started");
	}
	
	
	public void run() {
		randomInsertion();
	}
	
	public Proxy createProxy(int nb) {
		return createProxyOne();
		return null;
	}
	

	protected Proxy createProxyOne() {
		
		OriginInfo origin = createOriginInfo ("fakevision", "blublu2", "VisualObject");
		Proxy proxy = createNewProxy (origin, 0.9f);
		
		FeatureValue unknown = new UnknownValue();
		Feature feat1 = createFeatureWithUniqueFeatureValue ("obj_label", unknown);
		addFeatureToProxy (proxy, feat1);		
	
		return proxy;
	}
	
	

}
