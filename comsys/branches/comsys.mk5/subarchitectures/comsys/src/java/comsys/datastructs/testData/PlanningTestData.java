package comsys.datastructs.testData;

import java.util.Vector;
import comsys.utils.DialogueMoveUtils;
import comsys.datastructs.comsysEssentials.MoveType;


public class PlanningTestData extends TestData {

	
	boolean logging = false;
	
	String input;
	Vector<String> output;
	
	public PlanningTestData() {
		super();
	}
	
	public PlanningTestData(String input, Vector<String> output) {
		this.input = input;
		this.output= output;

	}
	
	public String getInput() {
		return input;
	}
	
	public Vector<String> getOutput() {
		return output;
	}
	

	public void setInput(String input) {
		this.input = input;
	}
	
	public void setOutput(Vector<String> output) {
		this.output = output;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[PLANNING TEST DATA] " +  str);
	}
}
