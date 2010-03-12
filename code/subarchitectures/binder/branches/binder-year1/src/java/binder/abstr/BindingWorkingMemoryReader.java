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


package binder.abstr;


import java.util.Vector;

import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.utils.BinderUtils;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;

/**
 * Abstract class for retrieving elements currently present in the working memory
 * of the binder
 * 
 * IMPORTANT NOTE: this component will only work if the UnionDiscretizer is also 
 * 				   activated in the CAST configuration
 * 
 * TODO: possibility to integrate handmade filters on binder WM changes
 * 
 * @author Pierre Lison
 * @version 09/09/2009 (started 15/08/2009)
 */
 
public class BindingWorkingMemoryReader extends ManagedComponent {

	// The current set of unions
	private Vector<Union> currentUnions ;
	
	
	/**
	 * Apply a filter on the binder WM to detect possible changes in the 
	 * best selected UnionConfiguration
	 */
	 
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
					extractUnionsFromConfig(config);
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		// initialize the set of current unions
		currentUnions = new Vector<Union>();
		
	}

	
	/**
	 * Return the (highest-likelihood) unions currently present in the binder
	 * working memory
	 * 
	 * @return a set of binding unions
	 */
	
	public Vector<Union> getUnions () {
		return currentUnions;
	}
	
	
	/**
	 * Extract the unions from the union configuration, and update the set of
	 * current unions accordingly
	 * 
	 * @param config the union configuration
	 */
	
	private void extractUnionsFromConfig (UnionConfiguration config) {
		currentUnions = new Vector<Union>();
		for (int i = 0; i < config.includedUnions.length ; i++) {
			currentUnions.add(config.includedUnions[i]);
		}
	}

	
}
