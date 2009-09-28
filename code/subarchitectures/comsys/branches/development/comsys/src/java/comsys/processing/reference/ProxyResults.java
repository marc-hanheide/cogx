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

import binder.autogen.specialentities.PhantomProxy;
import binder.autogen.specialentities.RelationProxy; 

// -------------------------------------------------------
// JAVA imports
// -------------------------------------------------------

import java.util.HashMap;
import java.util.Iterator;
import java.util.Vector;


public class ProxyResults {

	private HashMap<String, PhantomProxy> proxyMap = new HashMap<String, PhantomProxy>();
	private HashMap<String, String> IDtoNom  = new HashMap<String, String>();
	private HashMap<String, String> NomtoID  = new HashMap<String, String>();
	
	private Vector proxyRelations = new Vector();
	
	
	public void addPhantomProxy (PhantomProxy prx) { 
		proxyMap.put(prx.entityID, prx);
		NomtoID.put(prx.origin.address.id,prx.entityID);
		IDtoNom.put(prx.entityID, prx.origin.address.id);
	} // end method
	
	public void addProxies (ProxyResults results) { 
		for (Iterator<PhantomProxy> proxyIter = results.getProxies(); proxyIter.hasNext(); ) { 
			PhantomProxy prx = proxyIter.next();
			this.addPhantomProxy(prx);
		} // end for
	} // end method
		
	public void addRelationProxies (ProxyResults results) { 		
		for (Iterator<RelationProxy> rproxyIter = results.getRelationProxies(); rproxyIter.hasNext(); ) { 
			RelationProxy rprx = rproxyIter.next();
			this.addRelationProxy(rprx);
		} // end for
	} // end method
	

	public void addRelationProxy (RelationProxy rprx) { 
		proxyRelations.add(rprx);
	} // end method
	
	public Iterator<PhantomProxy> getProxies () { 
		return  proxyMap.values().iterator();
	} // end method
	
	public Iterator<RelationProxy> getRelationProxies() { 
		return (Iterator<RelationProxy>) proxyRelations.iterator();
	} // end method
	
	public PhantomProxy getPhantomProxyByNom (String nom) { return (PhantomProxy) proxyMap.get(nom); }
	
	
	
}
