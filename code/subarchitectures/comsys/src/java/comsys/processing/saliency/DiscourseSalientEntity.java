package comsys.processing.saliency;

public class DiscourseSalientEntity extends SalientEntity {
	
	String nomvar = "";
	
	public DiscourseSalientEntity (String concept, String nomvar, int recency) {
		super(concept);
		this.nomvar = nomvar;
		switch (recency) {
			case 0: score = 1.0f; break;
			case 1: score = 0.8f; break;
			case 2: score = 0.6f; break;
			case 3: score = 0.4f; break;
			default: score = 0.1f; break;
		}	
	}
	
	public String toString() {
		String result = concept + ", recency = " + score;
		return result;
	}
	
	public boolean isWellFormed() {
		return ((!concept.equals("")) && (nomvar != null)); 
	}

}
