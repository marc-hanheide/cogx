package comsys.datastructs.testData;

import java.util.Vector;
import comsys.datastructs.comsysEssentials.PhonString;

public class ParseSelectionTestData extends TestData {

	Vector<PhonString> inputs ;
	String transcription;
	String correctParse;
	
	public ParseSelectionTestData() {
		inputs = new Vector<PhonString>();
		transcription = "";
		correctParse = "";
	}
	
	public ParseSelectionTestData(Vector<PhonString> inputs, String transcription, String correctParse) {
		this.inputs = inputs;
		this.transcription = transcription;
		this.correctParse = correctParse;
	}
	
	public void setASRInputs(Vector<PhonString> inputs) {
		this.inputs = inputs;
	}
	
	public void setTranscription(String transcription) {
		this.transcription = transcription;
	}
	
	public String getTranscription() {
		return transcription;
	}
	
	public Vector<PhonString> getASRInputs() {
		return inputs;
	}
	
	public void setCorrectParse(String parse) {
		this.correctParse = parse;
	}
	
	public String getCorrectParse() {
		return correctParse;
	}
	
}
