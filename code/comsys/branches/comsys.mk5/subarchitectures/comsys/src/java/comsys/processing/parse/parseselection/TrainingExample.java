package comsys.processing.parse.parseselection;

import comsys.processing.parse.examplegeneration.GenerationUtils;
import comsys.datastructs.lf.*;
import comsys.lf.utils.LFUtils;

public class TrainingExample {

	String utterance;
	String recogUtterance;
	
	LogicalForm lf;
	
	public TrainingExample(String line) {
		int colonIndex = line.indexOf(":");	
		if (colonIndex >0) {
			String initUtterance = line.substring(0, colonIndex);
			String semantics = line.substring(colonIndex+1, line.length());
			init (initUtterance, initUtterance, semantics);
		}
	}
	
	public TrainingExample(String initUtterance, 
			String recogUtterance) {
		init(initUtterance, recogUtterance, "");
	}
	
	public void init(String initUtterance, 
			String recogUtterance, String semantics) {
		initUtterance = GenerationUtils.modifyCase(initUtterance);
		initUtterance = GenerationUtils.removeDisfluencies(initUtterance);
		this.utterance = initUtterance;
		recogUtterance = GenerationUtils.modifyCase(recogUtterance);
		recogUtterance = GenerationUtils.removeDisfluencies(recogUtterance);
		this.recogUtterance = recogUtterance;
		if (semantics.length() > 0) {
			LogicalForm lf = LFUtils.convertFromString(semantics);
			this.lf = GenerationUtils.removeDuplicateNominalsAndFeatures(lf);
		}
	}

	
	public LogicalForm getSemantics() {
		return lf;
	}
	
	public void setSemantics(String semantics) {
		if (semantics.length() > 0) {
			LogicalForm lf = LFUtils.convertFromString(semantics);
			this.lf = GenerationUtils.removeDuplicateNominalsAndFeatures(lf);
		}
	}
	
	public String getUtterance() {
		return utterance;
	}
	
	public String getRecognizedUtterance() {
		if (recogUtterance != null)
			return recogUtterance;
		else
			return utterance;
	}
	
	public void setRecognizedUtterance(String alteredUtt) {
		recogUtterance = alteredUtt;
	}
}
