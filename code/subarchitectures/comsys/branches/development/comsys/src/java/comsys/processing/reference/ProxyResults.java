//
//  ProxyResults.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

import binder.autogen.core.Proxy; 
import binder.autogen.core.RelationProxy; 

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.HashMap;


public class ProxyResults {

	
	private HashMap proxyMap = new HashMap();
	private HashMap nominalsMap = new HashMap();
	private Vector proxyRelations = new Vector();
	
	
	public void addProxy (Proxy prx, String nomVar) { 
		proxyMap.put(prx.entityId, prx);
		nominalsMap.put(nomVar,prx.entityId);
	} // end method
	
	public void addRelationProxy (RelationProxy rprx) { 
		proxyRelations.add(rprx);
	} // end method
	
	public Iterator<Proxy> getProxies () { 
		return (Iterator<Proxy>) proxyMap.values();
	} // end method
	
}
