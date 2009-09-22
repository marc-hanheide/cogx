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
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.specialentities.RelationProxy;

public class FakeHapticProxyGenerator extends AbstractProxyGenerator {

	String proxyOneId;
	String proxyTwoId;
	
	public void start () {
		log("Fake haptic proxy generator successfully started");
	}
	
	
	public void run() {
		randomInsertion();
	}
	
	
	public Proxy createProxy(int nb) {
		if (nb == 1) {
			return createProxyOne();
		}
		if (nb == 2) {
			return createProxyTwo();
		}
		
		if (nb == 3) {
			return createRelationProxy();
		}
		return null;
	}
	

	private Proxy createProxyTwo() {
		
		OriginInfo origin = createOriginInfo ("fakehaptic", "blibli", "GraspableObject");
		Proxy proxy = createNewProxy(origin, 0.65f);
		
		FeatureValue cylindrical = createStringValue ("cylindrical", 0.73f);
		Feature feat = createFeatureWithUniqueFeatureValue ("shape", cylindrical);
		addFeatureToProxy (proxy, feat);
		
		proxyOneId = proxy.entityID;
		
		log("Proxy one successfully created");
		return proxy;
	}
	
	

	private Proxy createProxyOne() {
		
		OriginInfo origin = createOriginInfo ("fakehaptic", "blibli2", "GraspableObject");
		Proxy proxy = createNewProxy (origin, 0.75f);
		
		FeatureValue spherical = createStringValue ("spherical", 0.67f);
		Feature feat1 = createFeatureWithUniqueFeatureValue("shape", spherical);
		addFeatureToProxy (proxy, feat1);
	
		BooleanValue trueval = createBooleanValue (true, 0.8f);
		BooleanValue falseval = createBooleanValue (false, 0.15f);
		FeatureValue[] vals = {trueval, falseval};
		Feature feat2 = createFeatureWithAlternativeFeatureValues("graspable", vals);
		addFeatureToProxy (proxy, feat2);
		
		proxyTwoId = proxy.entityID;
		
		log("Proxy two successfully created");

		return proxy;
	}
	

	private Proxy createRelationProxy() {
		
		AddressValue[] sources = new AddressValue[1];
		sources[0] = createAddressValue(proxyOneId, 0.9f);
		
		AddressValue[] targets = new AddressValue[1];
		targets[0] = createAddressValue(proxyTwoId, 0.91f);
		
		OriginInfo origin = createOriginInfo ("fakehaptic", "blibli3", "HapticRelation");
		RelationProxy proxy = createNewRelationProxy(origin, 0.81f, sources, targets);
		
		log("Relation proxy successfully created");

		return proxy;
	}
	
}
