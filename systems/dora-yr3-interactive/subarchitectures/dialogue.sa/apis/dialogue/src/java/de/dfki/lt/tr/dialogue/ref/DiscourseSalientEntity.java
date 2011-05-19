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
package de.dfki.lt.tr.dialogue.ref;

/**
 * Stores the information about the saliency of a discourse entity. The entity is identified by a
 * concept (ontological sort), and a (unique) nominal variable. Recency of mention controls the 
 * salience of this entity. 
 * 
 * @author 	Pierre Lison (plison@dfki.de)
 * @author	Geert-Jan M. Kruijff (gj@dfki.de)
 * @version 100608
 */


public class DiscourseSalientEntity extends SalientEntity {
	
	String nomvar = "";

	RecencySalienceScorer recencyScorer = new RecencySalienceScorer();
	
	public DiscourseSalientEntity (String concept, String nomvar, int recency) {
		super(concept);
		super.setSalienceScorer(recencyScorer);
		this.nomvar = nomvar;
		score = _salienceScorer.computeSaliency(recency);
	} // end constructor
	
	public String toString() {
		String result = concept + ", recency = " + score;
		return result;
	}
	
	public boolean isWellFormed() {
		return ((!concept.equals("")) && (nomvar != null)); 
	}

	/**
	 * Returns the nominal variable for the discourse entity
	 * @return String The stored nominal variable
	 */
	
	public String getNomVar () 
	{ 
		return nomvar; 
	}
	
	/**
	 * Updates the saliency stored for this discourse entity
	 * @param recency	The recency of mention for this entity
	 */
	
	public void setScore (int recency)
	{
		score = _salienceScorer.computeSaliency(recency);
	} // end setScore
	
	
	
} // end class
