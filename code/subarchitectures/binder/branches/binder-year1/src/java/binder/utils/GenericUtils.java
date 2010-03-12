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

package binder.utils;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.core.logging.ComponentLogger;

import binder.autogen.core.UnionConfiguration;



/**
 * Generic utility library
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 23/09/2009)
 */

public class GenericUtils {

	// flag to activate logging
	public static boolean logging = false;

	private static Logger logger = ComponentLogger.getLogger(GenericUtils.class);

	
	/**
	 * Extract text from a file (given its path)
	 * 
	 * @param aFile path of the file
	 * @return textual content of the file
	 */

	public static String getText(String aFile) {

		StringBuilder contents = new StringBuilder();

		try {

			BufferedReader input =  new BufferedReader(new FileReader(aFile));
			try {
				String line = null; 

				while (( line = input.readLine()) != null){
					contents.append(line);
					contents.append(System.getProperty("line.separator"));
				}
			}
			finally {
				input.close();
			}
		}
		catch (IOException ex){
			ex.printStackTrace();
		}

		return contents.toString();
	}

	
	public static int getIndex (UnionConfiguration[] array, UnionConfiguration obj) {
		
		for (int i = 0; i < array.length ; i++) {
			if (array[i].configProb == obj.configProb) {
				return i;
			}
		}
		
		return -1;
	}


	
	public static boolean isMoreRecent (CASTTime time1, CASTTime time2) {
		
		if (time1 != null && time2 != null) {
			
		if (time1.s > time2.s) {
			return true;
		}
		else if (time1.s < time2.s) {
			return false;
		}
		else if (time1.us > time2.us) {
			return true;
		}
		}
		else {
			return true;
		}
		return false;
	}

	
}
