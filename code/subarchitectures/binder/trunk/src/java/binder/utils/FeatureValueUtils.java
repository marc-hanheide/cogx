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

import cast.cdl.CASTTime;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.featvalues.BooleanValue;
import binder.autogen.featvalues.IntegerValue;
import binder.autogen.featvalues.StringValue;
import binder.autogen.featvalues.UnknownValue;

/**
 * Generic utility methods for the binder
 * 
 * @author Pierre Lison
 * @version 10/09/2009
 * @started 10/09/2009
 */

public class FeatureValueUtils {
	
	public static boolean LOGGING = true;
	

	public static UnknownValue createUnknownValue(float prob) {
		return new UnknownValue(prob, new CASTTime());
	}
	
	public static boolean hasUnknownValue (FeatureValue fv) {
		return (fv instanceof UnknownValue);
	}
	

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
		else {
	//		log("WARNING: Type of feature value is unknown / not comparable");
	//		log("feature value: " + fv.getClass().getSimpleName());
	//		log("object: " + o.getClass().getSimpleName());
		}
		return false;
	}
	
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
		if (fv instanceof BooleanValue && fv2 instanceof BooleanValue) {
			return ((BooleanValue)fv).val == (((BooleanValue)fv2).val);
		}
		if (fv instanceof UnknownValue && fv2 instanceof UnknownValue) {
			return true;
		}
		
		else {
	//		log("WARNING: Type of feature value is unknown / not comparable");
	//		log("feature value 1: " + fv.getClass().getSimpleName());
	//		log("feature value 2: " + fv2.getClass().getSimpleName());
		}
		return false;
	}
	
	
	public static String getString(FeatureValue fv) {
		if (fv.getClass().equals(StringValue.class)) {
			return ((StringValue)fv).val;
		}
		else {
			log("WARNING: type of feature value not a String");
		}
		return "";
	}
	
	public static boolean hasUnknownValues (Feature[] feats) {
		if (feats.length > 0) {
			if (feats[0].alternativeValues != null) {
				for (int i = 0 ; i < feats[0].alternativeValues.length ; i++) {
					if (hasUnknownValue(feats[0].alternativeValues[i])) {
						return true;
					}
				}
			}
		}
		return false;
	}
	
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
	

	private static void log(String s) {
		if (LOGGING) {
		System.out.println("[FeatureValueUtils] " + s);
		}
	}
	

}
