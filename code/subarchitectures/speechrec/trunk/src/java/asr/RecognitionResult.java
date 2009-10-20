
// =================================================================
//  Created by Pierre Lison on 10/12/06.
//  Copyright 2006 . All rights reserved.
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


package asr;

/**
 * A Nuance recognition result. 
 */

public class RecognitionResult {

	// True if something has been recognized, false otherwise
	boolean recognized ;
	
	// True if the recognition message is the consequence of a close SIP connection,
	// false otherwise
	boolean closedConnection = false ; 
	
	// The raw result as delivered by Nuance
	String fullResult = "" ;
	
	// The recognized string
	String recString = "" ;
	
	// Confidence value
	int confidence = 0;
	
	// Probability value
	int probability = 0 ;
	
	// IP address of the caller
	String origin = "" ;
	
	
	/*
	 * Create a new recognition result from a raw Nuance result
	 */
	public RecognitionResult (String result) {
		fullResult = result ;
		
		if (result.equals("closedConnection")) {
			closedConnection = true ;
			recognized = false ;
		}
		else if (!result.equals("unrecognized")) {
			
			recognized = true ;
			String[] lines = result.split("\n");
		
			recString = lines[2].split("=")[1].replaceAll("\"","") ;
			probability =  (Integer.valueOf(lines[3].split("=")[1])).intValue() ;
			confidence =  (Integer.valueOf(lines[4].split("=")[1])).intValue() ;
			
			String[] words = recString.split(" ");
			for (int i=0; i <words.length; i++) {
				if (words[i].equals("no")) {
					recString = recString.replace("no", "No");
				}
				if (words[i].equals("that's")) {
					recString = recString.replace("that's", "that is");
				}
			}
			
		}
		
		else {
			recognized = false ;
		}

	}
	
	/**
	 * Returns the recognized string
	 */
	public String getRecString () {return recString; }

	/**
	 * Returns the confidence value
	 */
	public int getConfidence () {return confidence; }

	/**
	 * Returns the probability value
	 */
	public int getProbability () {return probability; }

	/**
	 * Return true if a result has been recognized, false otherwise
	 */
	public boolean isRecognized() { return recognized; } 

	/**
	 * Return true if the SIP connection has been closed, false otherwise
	 */
	public boolean isConnectionClosed() { return closedConnection; } 
	
	/**
	 * Sets the origin (IP Address) of the caller
	 */
	public void setOrigin(String origin) {
		if (!origin.equals("")) { 
		String[] parts = origin.split(":") ;
		this.origin = parts[1].replace(">","") ;
		}
	}
	
	/**
	 * Return the origin (IP address) of the caller
	 */
	public String getOrigin() {return origin ; }
	
	/**
	 * Pretty print of the recognition result
	 */
	public String toString() {
		String text = "\n\n" ; 
		text = text + "Nuance Recognition results: \n" ;
		text = text + "----------\n" ;
		if (recognized) { 
			text = text + "Recognized string: " + recString + "\n" ;
			text = text + "Probability: " + probability + "\n";
			text = text + "Confidence value: " + confidence + "\n";
			if (!origin.equals(""))
				text = text + "Origin of the call " + origin + "\n";
			else
				text = text + "Origin of the call unknown "+ "\n" ;
		}
		else {
			text = text + "Sorry, utterance not recognized"+ "\n" ;
		}
		text = text + "----------"+ "\n" ;
		return text ;
	}
}
