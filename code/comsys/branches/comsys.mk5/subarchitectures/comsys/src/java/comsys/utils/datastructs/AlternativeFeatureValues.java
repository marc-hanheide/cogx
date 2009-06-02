package comsys.utils.datastructs;

import java.util.Vector;
import java.util.Enumeration;

public class AlternativeFeatureValues implements AbstractFeatureValue {

	String feat ;
	Vector<String> values;
	
	public AlternativeFeatureValues(String feat, Vector<String> values) {
		
		this.feat = feat.toLowerCase();
		this.values = values;
		
	}
	
	public String toString() {
		return "["+feat+ ": " + values.toString()+"]";
	}
	
	public String getFeat() { return feat; }
	
	public Vector<String> getValue() { return values; } 
	
	public boolean equals(AbstractFeatureValue fv) {
		return (this.feat.equals(fv.getFeat()) && this.values.contains(fv.getValue()));
	}
	
}
