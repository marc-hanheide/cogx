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
package de.dfki.lt.tr.cast.dialogue;

//=================================================================
// IMPORTS

// Java
import java.util.Hashtable;
import java.util.Vector;
import java.util.Enumeration;

// Dialogue API 
import de.dfki.lt.tr.dialogue.ref.*;

/**
 * 
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	100608
 */

public class SalienceModel extends Ice.ObjectImpl {

	Hashtable<SalientEntity, Float> distribution ;
	
	
	public SalienceModel() {
		distribution = new Hashtable<SalientEntity, Float>();
	}
	
	public void addSalientObjects(Vector<SalientEntity> objects) {
		
			float sum = 0.0f;
			for (Enumeration<Float> e = distribution.elements(); e.hasMoreElements();) {
				float score = e.nextElement().floatValue();
				sum += score;
			}
			for (Enumeration<SalientEntity> e = objects.elements(); e.hasMoreElements();) {
				SalientEntity entity = e.nextElement();
				float score = entity.getScore();
				sum += score;
			}

			// first renormalize the preexisting objects in distribution
			for (Enumeration<SalientEntity> o = distribution.keys() ; o.hasMoreElements() ;) {
				SalientEntity obj = o.nextElement() ;
				distribution.put(obj, new Float(obj.getScore()/sum));
			}
			
			// and add the new ones
			for (Enumeration<SalientEntity> o = objects.elements() ; o.hasMoreElements() ;) {
				SalientEntity obj = o.nextElement() ;
				distribution.put(obj, new Float(obj.getScore()/sum));
			}
	}
	
	public float getProbability (SalientEntity object) {
		if (distribution.containsKey(object)) {
			return distribution.get(object).floatValue();
		}
		return 0.0f;
	}
	
	public Hashtable<SalientEntity, Float> getDistribution() {
		return distribution;
	}
	
	public float getSaliencyOfDiscRef(String discref) {
		for (Enumeration<SalientEntity> e = distribution.keys(); e.hasMoreElements();) {
			SalientEntity entity = e.nextElement();
			if (entity.getClass().equals(DiscourseSalientEntity.class)) {
				DiscourseSalientEntity entity2 = (DiscourseSalientEntity) entity;
				if (entity2.getNomVar().equals(discref)) {
					return distribution.get(entity).floatValue();
				}
			}
		}		
		log("WARNING: discourse referent \"" + discref +"\" does not exist in salience model");
		return 0;
	}
	
	public String toString() {
		String result = "[";
		for (Enumeration<SalientEntity> o = distribution.keys() ; o.hasMoreElements() ;) {
			SalientEntity obj = o.nextElement() ;
			result += "(" + obj.getConcept() + ": " + distribution.get(obj) + ")";
			if (o.hasMoreElements()) {
				result += ", ";
			}
		}
		result += "]";
		return result;
	}
	
	private void log(String str) {
		System.out.println("[salience model] " + str);
	}
}
