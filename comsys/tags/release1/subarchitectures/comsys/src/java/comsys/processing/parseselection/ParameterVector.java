package comsys.processing.parseselection;

import java.util.Enumeration;
import java.util.Vector;

public class ParameterVector extends FeatureVector {
	
	public double initialValue = 5;
	
	public ParameterVector() {
		
	}
	
	public void initialise(Vector<String> featuresToConsider) {
		for (Enumeration<String> e = featuresToConsider.elements() ; e.hasMoreElements() ;) {
			String feature = e.nextElement();
			if (!features.containsKey(feature)) {
				features.put(feature, new Double(initialValue));
		//	log("new feature added in parameter vector: " + feature);
			}
		}		
	}
	
	public double innerProduct (FeatureVector fv) {
		
		double result = 0;
		
		for (Enumeration<String> e = features.keys() ; e.hasMoreElements(); ) {
			String feature = e.nextElement();
			double paramValue = features.get(feature).doubleValue();
			double featValue = fv.getFeatureValue(feature);
			result += paramValue * featValue ;
			
		/**	if (((paramValue * featValue) > 4.0) || (paramValue * featValue) < -4.0) {
				log("Feature: " + feature  + " --> " + (paramValue * featValue));
			} */
		}
	//	log("------------"); 
		return result;
	}
	
	public void update (FeatureVector fv_good, FeatureVector fv_bad) {
		
		for (Enumeration<String> e = features.keys() ; e.hasMoreElements(); ) {
			String feature = e.nextElement();
			double oldValue = features.get(feature).doubleValue();
			
			if (fv_good.getFeatureValue(feature) != 
				fv_bad.getFeatureValue(feature)) {
			
				double newValue = oldValue
								+ fv_good.getFeatureValue(feature)
								- fv_bad.getFeatureValue(feature);
				
				features.put(feature, new Double(newValue));
			}
		}
		
	}
	
	
	public String toString() {
		String result = "";
		for (Enumeration<String> e = features.keys(); e.hasMoreElements(); ) {
			String feat = e.nextElement();
			result += "w_{"+ feat + "} <- " + features.get(feat).doubleValue() + "\n";
		}
		return result;
	}
}
