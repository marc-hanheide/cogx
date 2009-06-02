package comsys.utils.datastructs;

public class FeatureValue implements AbstractFeatureValue {

	String feat ;
	String value;
	
	public FeatureValue(String feat, String value) {
		this.feat = feat.toLowerCase();
		this.value = value.toLowerCase();
	}
	
	public String toString() {
		return "["+feat+ ":" + value+"]";
	}
	
	public String getFeat() { return feat; }
	
	public String getValue() { return value; } 
	
	public boolean equals(AbstractFeatureValue fv) {
		return (this.feat.equals(fv.getFeat()) && this.value.equals(fv.getValue()));
	}
}
