package de.dfki.lt.hfc;

import java.util.ArrayList;
import java.util.Map;

import java.util.*;


/**
 * a class that can decode the variable bindings
 * into a results set with variable-to-resource mappings
 * 
 * @author (C) Hendrik Zender
 * @since JDK 1.6
 * @version Thu Aug 05 16:27:12 CET 2010
 */
public final class BindingTableDecoder {

	public static ArrayList<Map<String, String>> decode(BindingTable bt, TupleStore ts) {
	    ArrayList<Map<String, String>> decodedList = new ArrayList<Map<String,String>>();
	    // System.out.println("decode called, and arraylist initialized");
	    for (int[] intResLine : bt.table) {

	    	StringBuffer _outputLine = new StringBuffer();
	    	
	    	Map<String, String> _currDecodedLine = new HashMap<String, String>();
	    	for (Integer intVarName: bt.nameToPos.keySet()) {
	    		// the internal variable name is the first element (key) of the name2pos map
	    		// iterating over the keys gives us all 'bound' array positions to check
	    		// some logging:
	    		//System.out.println("BTD: current internal variable name: " + intVarName);
	    		//System.out.println("BTD: the external name of the variable is: " + bt.nameToExternalName.get(intVarName));
	    		//System.out.println("BTD: the associated array position is: " + bt.nameToPos.get(intVarName));
	    		//System.out.println("BTD: the current resolved line has the following int ID at this array position: " + intResLine[ bt.nameToPos.get(intVarName)]);
	    		//System.out.println("BTD: this int ID corresponds to the following resource: " + ts.getObject(intResLine[bt.nameToPos.get(intVarName)]));
	    		
    			String resource = ts.getObject(intResLine[bt.nameToPos.get(intVarName)]);
				String extVarName = bt.nameToExternalName.get(intVarName);
	    		
				_outputLine.append(extVarName + "->" + resource + " - ");
				_currDecodedLine.put(extVarName, resource);
			}
	    	decodedList.add(_currDecodedLine);
	    	// System.out.println(_outputLine);
	    }
	    //System.out.println("BTD: the decoded bindings list has a size of: " + decodedList.size());
		return decodedList;
	}
	
	
	
}
