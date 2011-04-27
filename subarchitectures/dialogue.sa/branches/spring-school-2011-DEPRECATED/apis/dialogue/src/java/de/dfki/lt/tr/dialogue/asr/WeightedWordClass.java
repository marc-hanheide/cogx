// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
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
import java.util.Vector;
import java.util.Hashtable;
import java.util.Enumeration;
import java.util.Iterator;

/**
 * Class of weighted words
 * 
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	100608
 */

public class WeightedWordClass {

	Hashtable<String, WeightedWord> weightedWords ;
	
	public WeightedWordClass() {
		weightedWords = new Hashtable<String,WeightedWord>();
	}
	
	public WeightedWordClass(Vector<WeightedWord> wwords) {
		this();
		for (Iterator<WeightedWord> e = wwords.iterator() ; e.hasNext() ;) {
			WeightedWord wword = e.next();
			weightedWords.put(wword.word, wword);
		}
	}
	
	public void addWeightedWord(WeightedWord wword) {
		weightedWords.put(wword.word, wword);
	}
	
	public WeightedWord getWeightedWord(String str) {
		if (weightedWords.containsKey(str)) {
			return weightedWords.get(str);
		}
		else {
			return null;
		}
	}
	
	public int size()  {
		return weightedWords.size() ;
	}
	
	public Enumeration<WeightedWord> getWeightedWords() {
		return weightedWords.elements() ;
	}
	
	public boolean hasWeightedWord(String str) {
		return weightedWords.containsKey(str);
	}
	
	public String toString() {
		String result = "";
		for (Enumeration<WeightedWord>  e = weightedWords.elements() ; e.hasMoreElements() ;) {
			WeightedWord word = e.nextElement();
			result += word.word + "~" + word.weight + "\n";
		}
		return result;
	}
	
} // end class
