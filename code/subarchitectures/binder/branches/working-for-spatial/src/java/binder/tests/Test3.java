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

package binder.tests;


import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

import binder.autogen.core.UnionConfiguration;

/**
 * Test 3: generate correct single and multi-proxy unions for random proxies and an associated
 * bayesian network
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 23/09/2009)
 */
public class Test3 extends AbstractTester{

	static int testNumber = 3;
	static String task = "Generate correct single- and multi-proxy unions for random proxies";
	
		
	public Test3 () {
		super(testNumber, task);
	}
	
	
	
	@Override
	public void start() {

		// if best selected UnionConfiguration has been updated in the WM, update the
		// reader accordingly
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					UnionConfiguration config = 
						getMemoryEntry(_wmc.address, UnionConfiguration.class);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
	}



	@Override
	public boolean performTest() {
		
		return false;
	}

	
}
