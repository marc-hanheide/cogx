//
//  AbstractProxyFactory.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference.proxyfactories;

import cast.cdl.CASTTime;
import comsys.datastructs.lf.LogicalForm;
import comsys.processing.reference.ProxyFactory;
import comsys.processing.reference.ProxyResults;

import binder.autogen.core.Feature;
import binder.autogen.featvalues.AddressValue;
import binder.utils.ProxyConstructor;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

public abstract class AbstractProxyFactory 
		implements ProxyFactory
	
{
	public static int idCounter = 0;

	abstract public String getSort (); 

	abstract public ProxyResults constructProxy (LogicalForm lf, CASTTime timestamp); 

	protected AddressValue[] createAddressValueArray (AddressValue addr) { 	
		AddressValue[] addressarray = new AddressValue[1];
		addressarray[0] = addr;
		return addressarray;
	} // end method
	
	protected Feature createSimpleFeature (String feature, String value, CASTTime timestamp) {
	    return ProxyConstructor.createFeatureWithUniqueFeatureValue (feature, ProxyConstructor.createStringValue (value, 1.0f, timestamp)); 
	} 
	
	
	protected String getEntityID() {
		idCounter++;	
		return "phantom" + idCounter;
	}
	
}
