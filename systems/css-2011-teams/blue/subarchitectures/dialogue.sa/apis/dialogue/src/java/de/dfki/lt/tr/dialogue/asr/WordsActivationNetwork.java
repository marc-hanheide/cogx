// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Pierre Lison (plison@dfki.de)
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

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.dialogue.asr;

//=================================================================
// IMPORTS
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;

/**
 * 
 * @author  Pierre Lison (plison@dfki.de)
 * @version	100608
 */

public class WordsActivationNetwork {

	Hashtable<String, WeightedWordClasses> activations;

	public WordsActivationNetwork(String fileName) {
		activations = new Hashtable<String,WeightedWordClasses>();
		try {
			BufferedReader in = new BufferedReader(new FileReader(fileName));
			String line = in.readLine() ;
			String concept = "";
			Vector<String> lines = new Vector<String>();
			while (line != null) {
				if (line.contains("{")) {
					String[] header = line.trim().split(" ");
					concept = header[0];
					lines = new Vector<String>();
				}
				else if (line.contains("}")) {
					WeightedWordClasses classes = new WeightedWordClasses(lines); 
					activations.put(concept, classes);
				}
				else {
					lines.add(line);
				}
				line = in.readLine();
			}
		}
		catch (Exception e) {
			System.out.println("ERROR, file "+ fileName + " specifying the word activation network doesnt exist!!");
			e.printStackTrace();
		}
	}
	
	public WeightedWordClasses getActivation(String entity) {
		return activations.get(entity);
	}
	
	public String toString() {
		String result = "";
		for (Enumeration<String> e = activations.keys() ; e.hasMoreElements() ;) {
			String key = e.nextElement();
			result += key + "{\n";
			result += activations.get(key).toString();
			result += "}\n\n";
		}
		return result;
	}
	
}
	

