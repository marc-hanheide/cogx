package comsys.processing.parseselection;

import java.util.StringTokenizer;

import comsys.datastructs.comsysEssentials.PhonString;
import comsys.datastructs.lf.LogicalForm;
import comsys.lf.utils.LFUtils;
import comsys.processing.parse.examplegeneration.GenerationUtils;

public class TrainingExample {

	PhonString expectedUtterance;
	int expectedUtteranceStringPos = 0;
	
//	PhonString recogUtterance;
	
	public String semantics;
	
	LogicalForm lf;
	
	static int phonIncr = 1;
	
	public TrainingExample(String line) {
		int colonIndex = line.indexOf(":");	
		if (colonIndex >0) {
			String initUtterance = line.substring(0, colonIndex);
			semantics = line.substring(colonIndex+1, line.length());
			init (initUtterance, initUtterance, semantics);
		}
	}
	
	public TrainingExample(String initUtterance, String recogUtterance) {
		init(initUtterance, recogUtterance, "");
	}
	
	public void init(String initUttString, 
			String recogUttString, String semantics) {
		
		expectedUtterance = new PhonString();
		
		expectedUtterance.wordSequence = initUttString;
		expectedUtterance.id = "utt-" + phonIncr;
		expectedUtteranceStringPos = 0;
		
	/**	recogUtterance = new PhonString();
		recogUtterance.wordSequence = GenerationUtils.modifyCase(recogUttString);
		recogUtterance.wordSequence = GenerationUtils.removeDisfluencies(recogUttString);
		recogUtterance.id = "recogUtt-" + phonIncr; */
		
		phonIncr++;

		if (semantics.length() > 0) {
			LogicalForm lf = LFUtils.convertFromString(semantics);
			this.lf = GenerationUtils.removeDuplicateNominalsAndFeatures(lf);
		}
		
		int length = 0;
		StringTokenizer tokenizer = new StringTokenizer(initUttString);
		while (tokenizer.hasMoreTokens()) {
			tokenizer.nextToken();
			length++;
		}
		expectedUtterance.length = length;
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
	
	public PhonString getExpectedUtterance() {
		return expectedUtterance;
	}
	
	
	public String getIncrementalUtterance() {
		StringTokenizer tokenizer = new StringTokenizer(expectedUtterance.wordSequence);
		String formattedUtt = "";
		int incr = 0;
		while (tokenizer.hasMoreTokens() && 
				expectedUtteranceStringPos <= expectedUtterance.length && 
				incr <= expectedUtteranceStringPos) {
			formattedUtt += tokenizer.nextToken();
			if (tokenizer.hasMoreTokens()) formattedUtt += " ";
			incr++;
		}
		return formattedUtt;
	}
	
	
	public String getFormattedUtterance() {
		StringTokenizer tokenizer = new StringTokenizer(expectedUtterance.wordSequence);
		String formattedUtt = "";
		int incr = 1;
		while (tokenizer.hasMoreTokens()) {
			formattedUtt += tokenizer.nextToken();
			if (expectedUtteranceStringPos == incr) { formattedUtt += " * "; }
			if (tokenizer.hasMoreTokens()) formattedUtt += " ";
			incr++;
		}
		return formattedUtt;
	}
	
/**	public String getRecognizedUtterance() {
		if (recogUtterance != null)
			return recogUtterance;
		else
			return utterance;
	}
	
	public void setRecognizedUtterance(String alteredUtt) {
		recogUtterance = alteredUtt;
	} */
}
