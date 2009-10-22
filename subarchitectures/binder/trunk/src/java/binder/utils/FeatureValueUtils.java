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

import org.apache.log4j.Logger;

import cast.cdl.CASTTime;
import cast.core.logging.ComponentLogger;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.FloatValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;
import binder.constructors.ProxyConstructor;

/**
 * Generic utility methods to manipulate feature values
 * 
 * @author Pierre Lison
 * @version 10/09/2009 (started 10/09/2009)
 */

public class FeatureValueUtils {

	// flag to activate error logging
	public static boolean ERRLOGGING = true;

	// flag to activate logging
	public static boolean LOGGING = false;
	
	private static Logger logger = ComponentLogger.getLogger(FeatureValueUtils.class);


	// ================================================================= 
	// UNKNOWN VALUES METHODS   
	// ================================================================= 
	
	
	/**
	 * Create a new unknown feature value
	 * 
	 * @param prob existence probability of the value
	 * @return the feat value
	 */
	
	public static UnknownValue createUnknownValue(float prob) {
		return new UnknownValue(prob, new CASTTime(0,0));
	}
	
	/**
	 * Return true if fv is an unknown value
	 * 
	 * @param fv the feature value
	 * @return true if unknown value, false otherwise
	 */
	
	public static boolean isUnknownValue (FeatureValue fv) {
		return (fv instanceof UnknownValue);
	}
	
	

	/**
	 * Check whether the feature array contain any unknown values
	 * 
	 * @param feats the feature array
	 * @return true if there is at least one unknown feature value, false otherwise
	 */
	
	public static boolean hasUnknownValues (Feature[] feats) {
		if (feats.length > 0) {
			if (feats[0].alternativeValues != null) {
				for (int i = 0 ; i < feats[0].alternativeValues.length ; i++) {
					if (isUnknownValue(feats[0].alternativeValues[i])) {
						return true;
					}
				}
			}
		}
		return false;
	}
	
	
	// ================================================================= 
	// FEATURE VALUE COMPARISON METHODS   
	// ================================================================= 
	
	
	/**
	 * Check whether the two feature values have the same content
	 * 
	 * @param fv first feature value
	 * @param fv2 second feature value
	 * @return true is the two feat values match, false otherwise
	 */
	
	public static boolean haveEqualValue (FeatureValue fv, FeatureValue fv2) {
		if (fv instanceof StringValue && fv2 instanceof StringValue) {
			return ((StringValue)fv).val.equals(((StringValue)fv2).val);
		}
		if (fv instanceof AddressValue && fv2 instanceof AddressValue) {
			return ((AddressValue)fv).val.equals(((AddressValue)fv2).val);
		}
		if (fv instanceof IntegerValue && fv2 instanceof IntegerValue) {
			return ((IntegerValue)fv).val == (((IntegerValue)fv2).val);
		}
		
		
		// TODO:  CORRECT THIS!!! (quick fix for saliency values)
		if (fv instanceof FloatValue && fv2 instanceof FloatValue) {
			return true;
		}
		
		
		if (fv instanceof BooleanValue && fv2 instanceof BooleanValue) {
			return ((BooleanValue)fv).val == (((BooleanValue)fv2).val);
		}
		if (fv instanceof UnknownValue && fv2 instanceof UnknownValue) {
			return true;
		}
		
		return false;
	}
	


	/**
	 * check whether the value of fv equals o
	 * 
	 * @param fv the feature value
	 * @param o the object
	 * @return true is the feature value content and the object match, false otherwise
	 */

	public static boolean hasValue (FeatureValue fv, Object o) {
		
		if (fv instanceof StringValue && o instanceof String) {
			return ((StringValue)fv).val.equals(((String)o));
		}
		if (fv instanceof AddressValue && o instanceof String) {
			return ((AddressValue)fv).val.equals(((String)o));
		}
		if (fv instanceof BooleanValue && o instanceof Boolean) {
			return ((BooleanValue)fv).val == (((Boolean)o).booleanValue());
		}
		if (fv instanceof IntegerValue && o instanceof Integer) {
			return ((IntegerValue)fv).val == (((Integer)o).intValue());
		}
		if (fv instanceof FloatValue && o instanceof Float) {
			return ((FloatValue)fv).val == (((Float)o).floatValue());
		}
		
		return false;
	}
	
	
	public static FeatureValue mergeFeatureValues (FeatureValue fv1, FeatureValue fv2) {
		
		if (fv1 instanceof FloatValue && fv2 instanceof FloatValue) {
			float mean = (((FloatValue)fv1).val + ((FloatValue)fv2).val) / 2.0f;
			float meanProb = (fv1.independentProb + fv2.independentProb) / 2.0f;
			FloatValue mergedFV = ProxyConstructor.createFloatValue(mean, meanProb);
			ProxyConstructor.setTimeStamp(mergedFV, fv2.timeStamp);
			return mergedFV;
		}
		
		if (FeatureValueUtils.haveEqualValue(fv1, fv2)) {
			return fv1;
		}
		
		else {
			errlog("WARNING: feature merging for these type of values not implemented yet!");
		}
		
		return new FeatureValue();
	}
	
	// ================================================================= 
	// FEATURE VALUE CLONING METHODS   
	// ================================================================= 

	 
	/**
	 * Return a new feature value with exactly the same content and probability as fv
	 * 
	 * @param fv the existing feature value
	 * @return the new feature value
	 */
	
	public static FeatureValue cloneFeatureValue(FeatureValue fv) {
		
		if (fv instanceof StringValue) {
			return new StringValue(fv.independentProb, fv.timeStamp, ((StringValue) fv).val);
		}
		else if (fv instanceof AddressValue) {
			return new AddressValue(fv.independentProb, fv.timeStamp, ((AddressValue) fv).val);
		}
		else if (fv instanceof IntegerValue) {
			return new IntegerValue(fv.independentProb, fv.timeStamp, ((IntegerValue) fv).val);
		}
		else if (fv instanceof FloatValue) {
			return new FloatValue(fv.independentProb, fv.timeStamp, ((FloatValue) fv).val);
		}
		else if (fv instanceof BooleanValue) {
			return new BooleanValue(fv.independentProb, fv.timeStamp, ((BooleanValue) fv).val);
		}
		else if (fv instanceof UnknownValue) {
			return new UnknownValue (fv.independentProb, fv.timeStamp);
		}
		else {
			log("WARNING: feature value not convertible to a string");
			return new FeatureValue(fv.independentProb, fv.timeStamp);
		}
	}
	

	// ================================================================= 
	// MAXIMUM SEARCH METHODS   
	// ================================================================= 

	
	/**
	 * Get the feature value with the highest probility in the feature
	 * 
	 * @param feat the feature
	 * @return the max-probability feature value
	 */
	
	public static FeatureValue getMaxFeatureValue (Feature feat) {
		
		float maxProb = 0.0f;
		FeatureValue maxFeatValue = new FeatureValue();
		
		for (int i = 0 ; i < feat.alternativeValues.length ; i++) {
			
			if (feat.alternativeValues[i].independentProb > maxProb) {
				maxProb = feat.alternativeValues[i].independentProb;
				maxFeatValue = feat.alternativeValues[i];
			}
		}
		
		return maxFeatValue;
	}
	

	// ================================================================= 
	// STRING FORMATTING METHODS   
	// ================================================================= 

	
	/**
	 * Return a string representation of the feature value
	 * 
	 * @param fv the feature value
	 * @return the string-format representation
	 */
	public static String toString (FeatureValue fv){
		if (fv instanceof StringValue) {
			return ((StringValue)fv).val;
		}
		else if (fv instanceof AddressValue) {
			return ((AddressValue)fv).val;
		}
		else if (fv instanceof IntegerValue) {
			return (""+((IntegerValue)fv).val);
		}
		else if (fv instanceof FloatValue) {
			return (""+((FloatValue)fv).val);
		}
		else if (fv instanceof BooleanValue) {
			return (""+((BooleanValue)fv).val);
		}
		else if (fv instanceof UnknownValue) {
			return "unknown";
		}
		else {
			log("WARNING: feature value not convertible to a string");
		}
		return "";
	}
	
	
	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 

	
	/**
	 * Logging
	 * @param s
	 */
	private static void log(String s) {
		if (LOGGING) {
		logger.debug ("[FeatureValueUtils] " + s);
		}
	}
	
	private static void errlog (String s) {
		if (ERRLOGGING)
		logger.debug ("[FeatureValueUtils] " + s);
	}

}
