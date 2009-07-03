package comsys.processing.saliency;

public abstract class SalientEntity {

	String concept = "" ;
	float score;
	
	public SalientEntity (String concept) {
		this.concept = concept;
	}
	
	public void setConcept (String concept) {
		this.concept = concept;
	}
	
	public String getConcept() { return concept; }
	
	public float getScore() { return score; }
}
