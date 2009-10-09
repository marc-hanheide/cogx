package comsys.processing.parseselection;

import java.util.Vector;
import java.util.Hashtable;
import java.util.Enumeration;

public class FeatureVector {

	public boolean logging = true;
	
	protected Hashtable<String,Double> features;
	
	public FeatureVector() {
		features = new Hashtable<String,Double>();
	}
	
	public void addFeature(String feature, double value) {
		if (! features.containsKey(feature)) {
			features.put(feature, new Double(value));
		}
		else {
			features.put(feature, new Double(features.get(feature)+value));
		}
	}
	
	public void incrementFeatureValue(String feature) {
		if (! features.containsKey(feature)) {
			setFeatureValue(feature, 1);
		}
		else {
			Double oldValue = features.get(feature);
			setFeatureValue(feature, oldValue.doubleValue() + 1);
		}
	}
	
	public void setFeatureValue(String feature, double value) {
			features.put(feature, new Double(value));
	}
	
	
	public double getFeatureValue(String feature) {
		if (features.containsKey(feature)) {
			return features.get(feature).doubleValue();
		}
		return 0;
	}
	
	public Enumeration<String> getFeatures() {
		return features.keys();
	}
	
	public int size() {
		return features.size();
	}
	
	public boolean hasFeature(String feat) {
		return features.containsKey(feat);
	}
	
	
	public boolean isSimilarToFV(FeatureVector fv) {
		if (size() != fv.size()) {
			return false;
		}
		else {
			for (Enumeration<String>  e = features.keys() ; e.hasMoreElements() ;) {
				String feat = e.nextElement();
				if (!fv.hasFeature(feat)) {
					return false;
				}
				else {
					double value1 = features.get(feat).doubleValue();
					double value2 = fv.getFeatureValue(feat);
					if (value1 != value2) {
						return false;
					}
				}
			}
			for (Enumeration<String>  e = fv.getFeatures() ; e.hasMoreElements() ;) {
				String feat = e.nextElement();
				if (!hasFeature(feat)) {
					return false;
				}
				else {
					double value1 = fv.getFeatureValue(feat); 
					double value2 = features.get(feat).doubleValue();
					if (value1 != value2) {
						return false;
					}
				}
			}
		}
		log("size of feature vector 1: " + features.size());
		log("size of feature vector 2: " + fv.size());
		
		return true;
	}
	
	protected void log(String str) {
		if (logging) {
			System.out.println("[feature vector] " + str);
		}
	}
	
	
	
}
