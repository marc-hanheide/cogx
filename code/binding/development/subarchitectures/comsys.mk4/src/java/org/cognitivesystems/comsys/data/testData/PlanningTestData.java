package org.cognitivesystems.comsys.data.testData;

import java.util.Vector;
import org.cognitivesystems.comsys.general.DialogueMoveUtils;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;


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
