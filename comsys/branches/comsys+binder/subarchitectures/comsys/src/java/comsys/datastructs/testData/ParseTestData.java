package comsys.datastructs.testData;

import java.util.Vector;

public class ParseTestData extends TestData {


	Vector<String> parses ;
	
	public ParseTestData() {
		this.string = "";
		parses = new Vector<String>();
	}
	
	public ParseTestData(String string, Vector<String> parses) {
		this.string = string;
		this.parses = parses;
	}
	
	public void setParses(Vector<String> parses) {
		this.parses = parses;
	}
	
	public Vector<String> getParses() {
		return parses;
	}
}
