//
//  ProxyFactory.java
//  
//
//  Created by Geert-Jan Kruijff on 9/21/09.
//  Copyright 2009 __MyCompanyName__. All rights reserved.
//

package comsys.processing.reference;


import cast.cdl.CASTTime;
import binder.autogen.core.Proxy; 
import comsys.datastructs.lf.LogicalForm; 



public interface ProxyFactory {

	public String getSort (); 
	
	public ProxyResults constructProxy (LogicalForm lf, CASTTime timestamp); 
	
}


