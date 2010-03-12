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

import java.util.Map;
import java.util.Random;

import binder.abstr.ProxyWriter;
import binder.autogen.core.Proxy;


/**
 * Abstract class to generate and insert proxies into the binder
 * 
 * @author Pierre Lison
 * @version 22/09/2009 (started 05/08/2009)
 */

public abstract class AbstractProxyGenerator extends ProxyWriter {

	// Number of proxies to create
	protected int nbOfProxiesToCreate = 0;
	
	// Whether to create the proxies in the standard or inverted order
	protected boolean reverted = false;

	// Whether to make pauses (of about 1 s.) between the proxy insertions
	protected boolean pauses = false;

	
	/**
	 * Configuration
	 */
	@Override
	public void configure(Map<String, String> _config) {
		if (_config.containsKey("--nbproxies")) {
			nbOfProxiesToCreate = Integer.parseInt(_config.get("--nbproxies"));
		} 
		if (_config.containsKey("--reverted")) {
			reverted = Boolean.parseBoolean(_config.get("--reverted"));
		} 
		if (_config.containsKey("--pauses")) {
			pauses = Boolean.parseBoolean(_config.get("--pauses"));
		} 
	}
	
	
	/**
	 * Create proxies and inserts them into the binder WM
	 */
	public void randomInsertion() {	
		Random rand = new Random();
		if (nbOfProxiesToCreate > 0) {
			
			// Loop on the proxies to create
			for (int i = 1 ; i < (nbOfProxiesToCreate +1) ; i++) {
				
				// Make a short pause
				if (pauses) {
					sleepComponent(500 + rand.nextInt(2000));
				}
				
				// Create the proxy
				Proxy p;
				if (!reverted) {
					p = createProxy(i);
				}
				else {
					p = createProxy(nbOfProxiesToCreate-i+1);
				}
				
				// And add it to the working memory
				addProxyToWM(p);

			}	
		}
	}
	
	
	/**
	 *  Create the proxy nb (nb being an integer between 1 and nbOfProxiesToCreate)
	 *  
	 * @param nb index of the proxy to create
	 * @return the created proxy
	 */
	protected abstract Proxy createProxy (int i) ;
	
}
