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

import binder.abstr.BindingWorkingMemoryWriter;
import binder.autogen.core.Proxy;

public abstract class AbstractProxyGenerator extends BindingWorkingMemoryWriter {

	protected int nbOfProxiesToCreate = 0;
	protected boolean reverted = false;
	protected boolean pauses = false;
	
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
	
	
	
	public void randomInsertion() {	
		Random rand = new Random();
		if (nbOfProxiesToCreate > 0) {
			for (int i = 1 ; i < (nbOfProxiesToCreate +1) ; i++) {
				if (pauses) {
					sleepComponent(500 + rand.nextInt(1000));
				}
				Proxy p;
				if (!reverted) {
					p = createProxy(i);
				}
				else {
					p = createProxy(nbOfProxiesToCreate-i+1);
				}
				addProxyToWM(p);

			}	
		}
	}
	
	
	
	protected abstract Proxy createProxy (int i) ;
	
}
