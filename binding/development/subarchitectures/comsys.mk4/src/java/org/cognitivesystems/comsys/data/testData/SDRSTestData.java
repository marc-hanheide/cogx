package org.cognitivesystems.comsys.data.testData;

import java.util.Vector;
import org.cognitivesystems.comsys.general.DialogueMoveUtils;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;


public class SDRSTestData extends TestData {

	
	boolean logging = false;
	
	Vector<String> discourse;
	
	
	public SDRSTestData() {
		super();
	}
	
	public SDRSTestData(Vector<String> discourse) {
		this.discourse = discourse;

	}
	
	
	public Vector<String> getDiscourse() {
		return discourse;
	}
	

	public void setDiscourse(Vector<String> discourse) {
		this.discourse = discourse;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[SDRS TEST DATA] " +  str);
	}
}
