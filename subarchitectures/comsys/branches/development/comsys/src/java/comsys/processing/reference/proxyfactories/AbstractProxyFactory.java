//
//  AbstractProxyFactory.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference.proxyfactories;

// -------------------------------------------------------
// BINDER imports
// -------------------------------------------------------

import binder.abstr.BindingWorkingMemoryWriter;
import binder.autogen.core.Proxy; 

// -------------------------------------------------------
// COMSYS imports
// -------------------------------------------------------

import comsys.datastructs.lf.LogicalForm; 
import comsys.processing.reference.ProxyFactory;
import comsys.processing.reference.ProxyResults;

public abstract class AbstractProxyFactory 
		extends BindingWorkingMemoryWriter
		implements ProxyFactory
	
{

	abstract public String getSort (); 

	abstract public ProxyResults constructProxy (LogicalForm lf); 

	protected AddressValue[] createAddressValueArray (AddressValue addr) { 	
		AddressValue[] addressarray = new AddressValue[1];
		addressarray[0] = addr;
		return addressarray;
	} // end method
	
	protected Feature createSimpleFeature (String feature, String value) {
	    return createFeatureWithUniqueFeatureValue (feature, createStringValue (value, 1.0f)); 
	} 
	
	
}
