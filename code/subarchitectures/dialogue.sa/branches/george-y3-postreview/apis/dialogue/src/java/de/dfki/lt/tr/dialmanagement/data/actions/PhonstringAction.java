// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (plison@ifi.uio.no)                                                                
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


package de.dfki.lt.tr.dialmanagement.data.actions;

import java.util.Collection;
import java.util.HashMap;
import java.util.StringTokenizer;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.dialmanagement.arch.DialogueException;
import de.dfki.lt.tr.dialmanagement.utils.FormulaUtils;
import de.dfki.lt.tr.dialmanagement.utils.PolicyUtils;


/**
 * Action to produce a (possibly underspecified) phonological string
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 22/12/2010
 *
 */
public class PhonstringAction extends AbstractAction {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	// the initial (possibly underspecified) string content
	String initContent;
	
	// the instantiated (eliminating underspecification) content
	String instantiatedContent;
	
	
	/**
	 * Create a new phonstring action with an identifier
	 * and a string content
	 * 
	 * @param id the identifier
	 * @param content the string content
	 */
	public PhonstringAction(String id, String content) {
		super(id);
		initContent = content;
		instantiatedContent = content;
	}

	
	/**
	 * Returns a logical representation of the (instantiated) action
	 */
	public dFormula asFormula () {
		return new ElementaryFormula(0, instantiatedContent);
	}
	
	

	/**
	 * Returns true if the action is underspecified, false otherwise
	 */
	@Override
	public boolean isUnderspecified() {
		return initContent.contains("%");
	}
	
	
	/**
	 * Returns the list of underspecified arguments in the phonstring
	 */
	@Override
	public Collection<String> getUnderspecifiedArguments() {
		return PolicyUtils.getUnderspecifiedArguments(new ElementaryFormula(0, initContent));
	}


	/**
	 * If the action is underspecified, and assuming the underspecification
	 * can be resolved using the provided arguments, replace the variable names
	 * by the actual values provided in the arguments
	 * 
	 * If the variable resolution fails, an exception is thrown.
	 * 
	 * @param arguments the arguments to fill
	 * @throws DialogueException if the variable resolution fails
	 */
	public void fillArguments (HashMap<String,dFormula> arguments) throws DialogueException {
		
			String[] initSplit = initContent.split("%");
			
			if (initSplit.length == 2) {
						
				String varName = "";
				for (int i = 0 ; i < initSplit.length ; i++) {
					StringTokenizer t = new StringTokenizer(initSplit[i]);
					varName = removePunctuation(t.nextToken().trim());
				}
				debug("varName to replace: " + varName);

				if (arguments.containsKey(varName)) {
					instantiatedContent =  initContent.replace("%"+varName, FormulaUtils.getString(arguments.get(varName)));
				}
			}
			
			if (instantiatedContent.contains("%")) {
				throw new DialogueException("ERROR, action could not be instantiated");
			}
		
	}

	
	
	/**
	 * Removes the punctuation of the given string
	 * @param s the string
	 * @return the cleaned up string
	 */
	private static String removePunctuation (String s) {
		return s.replace(".","").replace(",", "").replace("!","").replace("?", "").replace(":","").replace(";", "").trim();
	}
	

	/**
	 * Returns a string representation of the (instantiated) action
	 */
	@Override
	public String toString() {
			return instantiatedContent;
	}
	
	
	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[phonstringaction] " + s);
		}
	}
	
	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[phonstringaction] " + s);
		}
	}

	
}




