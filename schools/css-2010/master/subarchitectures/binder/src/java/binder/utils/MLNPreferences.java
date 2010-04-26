package binder.utils;

/**
 * 
 * @author Carsten Ehrler (carsten.ehrler@gmail.com)
 * 
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
	
	public void setGeneratedMLNFile (String generatedMLNFile) {
		this.generatedMLNFile = generatedMLNFile;
	}
	
	public String getGeneratedMLNFile () {
		return generatedMLNFile;
	}

	private float greediness;
	 
	private float outcome;
	
	private String file_correlations;
	
	private String file_predicates;
	
	private String generatedMLNFile;
	
	private boolean performTracking = false;
	
	public void activateTracking() {
		performTracking = true;
	}
	
	public boolean isTrackingActivated() {
		return performTracking;
	}
	
	public MLNPreferences() {
		greediness = -3.5f;
		outcome = -1f;
		file_correlations = markovlogicDir + "tracking/tracking-objects.mln"; 
		file_predicates = markovlogicDir + "tracking/correlations_predicates.mln";
		generatedMLNFile = markovlogicDir + "tracking.mln";
	}

	public MLNPreferences(float greediness, float outcome,
			String fileCorrelations, String filePredicates) {
		assert fileCorrelations != null;
		assert filePredicates != null;
		
		this.greediness = greediness;
		this.outcome = outcome;
		file_correlations = fileCorrelations;
		file_predicates = filePredicates;
	}
}
