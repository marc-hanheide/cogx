
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


package de.dfki.lt.tr.dialmanagement.utils;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;


/**
 * Generic utility for reading and writing text files
 * 
 * @author Pierre Lison (plison@ifi.uio.no)
 * @version 09/10/2010
 */

public class FileUtils {

	// logging and debugging
	public static boolean LOGGING = true;
	public static boolean DEBUG = false;
	
	private static String NEWLINE = "\n"; 

	
	/**
	 * Return the full content of a text file as a string
	 * 
	 * @param filename the name of the file to read
	 * @return the string with the file content
	 * @throws IOException if file is not found or not readable
	 */
	public static String readfile(String filename) throws IOException {
		
		StringBuilder text = new StringBuilder();
		FileInputStream fstream;
		fstream = new FileInputStream(filename);
		DataInputStream in = new DataInputStream(fstream);
		BufferedReader br = new BufferedReader(new InputStreamReader(in));
		String strLine;
			while ((strLine = br.readLine()) != null)   {
				text.append(strLine);
				text.append(NEWLINE);
			}
		return text.toString();
	}

	
	/**
	 * Write a string to a file (erasing the existing content)
	 * 
	 * @param filename filename of the file to write
	 * @param text the text to write
	 * @throws IOException if the file is not found or not writable
	 */
	public static void writeFile(String filename, String text) throws IOException {
			FileWriter fstream = new FileWriter(filename);
			BufferedWriter out = new BufferedWriter(fstream);
			out.write(text);
			out.close();
	}
	
	


	/**
	 * Logging
	 * @param s
	 */
	private static void log (String s) {
		if (LOGGING) {
			System.out.println("[fileutils] " + s);
		}
	}

	/**
	 * Debugging
	 * @param s
	 */
	private static void debug (String s) {
		if (DEBUG) {
			System.out.println("[fileutils] " + s);
		}
	}
	
}