package binder.utils;

/**
 * 
 * @author Carsten Ehrler (carsten.ehrler@gmail.com)
 * 
 * {@link #MLNPreferences()}
 */
public class MLNPreferences {
	
	public static String markovlogicDir = "subarchitectures/binder/markovlogic/";
	
	public float getGreediness() {
		return greediness;
	}

	public void setGreediness(float greediness) {
		this.greediness = greediness;
	}

	public float getOutcome() {
		return outcome;
	}

	public void setOutcome(float outcome) {
		this.outcome = outcome;
	}

	public String getFile_correlations() {
		return file_correlations;
	}

	public void setFile_correlations(String fileCorrelations) {
		file_correlations = fileCorrelations;
	}

	public String getFile_predicates() {
		return file_predicates;
	}

	public void setFile_predicates(String filePredicates) {
		file_predicates = filePredicates;
	}

	private float greediness;
	
	private float outcome;
	
	private String file_correlations;
	
	private String file_predicates;
	
	public MLNPreferences() {
		greediness = -1.5f;
		outcome = -1f;
		file_correlations = markovlogicDir + "grouping/correlations.mln"; 
		file_predicates = markovlogicDir + "grouping/correlations_predicates.mln";
	}

	public MLNPreferences(float greediness, float outcome,
			String fileCorrelations, String filePredicates) {
		super();
		this.greediness = greediness;
		this.outcome = outcome;
		file_correlations = fileCorrelations;
		file_predicates = filePredicates;
	}
}
