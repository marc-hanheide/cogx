package comsys.processing.parseselection;

import java.util.Vector;
import java.util.Enumeration;
import java.util.Hashtable;

import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;

public class AveragedPerceptron {

	boolean logging = true;
	
	int T = 1;
	
	public int countUpdates = 0;
	
	Decoder decoder ;
	ParameterVector params;

	Vector<Integer> statistics;
	
	boolean hasBeenUpdated = false;
	
	public AveragedPerceptron(ParameterVector params) {
		this.params = params;
		decoder = new Decoder(params);
		statistics = new Vector<Integer>();
	}
	
	public void setDecoder(Decoder decoder) {
		this.decoder = decoder;
	}
	
	public void setNumberOfIterations(int T) {
		this.T = T;
	}
	
	public boolean hasBeenUpdated() {
		return hasBeenUpdated;
	}
	
	public ParameterVector learn(Vector<TrainingExample> examples) {
			
		for (int i= 0; i < T ; i++) {
			
			for (Enumeration<TrainingExample> e = examples.elements(); 
			e.hasMoreElements(); ) {		
				
				TrainingExample example = e.nextElement();
				
		//		log("Training example: \"" + example.getFormattedUtterance() + "\"");

				LogicalForm bestParse = decoder.getBestParse(example.getExpectedUtterance(), example.expectedUtteranceStringPos);
				
				if (!LFUtils.isLf1IncludedInLf2(bestParse, example.getSemantics())) {
					
					log("Decoded best parse does not match expected parse --> updating parameter vector");
					log("Decoded parse: " +  LFUtils.lfToString(bestParse));
					
					LogicalForm bestParseSem = 
						decoder.getBestParseWithIncludedSemantics(example.getExpectedUtterance(), 
								example.getSemantics()); 
					
					if (bestParseSem != null) {
						FeatureExtractionFromLF extraction = 
							new FeatureExtractionFromLF(bestParseSem);
						FeatureVector fv_good = extraction.extractSemanticFeatures();
						FeatureVector fv_bad = decoder.getFV(bestParse.logicalFormId);
						
						if (!fv_good.isSimilarToFV(fv_bad)) {
							params.update(fv_good, fv_bad);
							countUpdates++;
							log("Parameter vector successfully updated");
							statistics.add(new Integer(2));
						}
						else {
							log("Feature vectors of both parses are similar, unable to update parameter vector");
							log("(in other words: the current feature representation is too crude)");
							log("Logical form chosen by current model: " + LFUtils.lfToString(bestParse));
							statistics.add(new Integer(1));
						}
						
						hasBeenUpdated = true;
					}
					else {
						log("WARNING: A golden standard partial parse cannot be found");
					}
					
				}
				else {
					statistics.add(new Integer(0));
					hasBeenUpdated = false;
					log("Decoded best parse already matches expected parse --> keep parameter vector as such");
				}
				
			}
		}	
		return params;
	}
	
	
	public ParameterVector learn(TrainingExample example) {
		Vector<TrainingExample> examples = new Vector<TrainingExample>();
		examples.add(example);
		return learn(examples);
	}
	
	public Vector<Integer> getStatistics() {
		return statistics;
	}
	
	private void log(String str) {
		if (logging)
			System.out.println("\033[31m[Perceptron] \t" + str + "\033[0m");
	}
}
