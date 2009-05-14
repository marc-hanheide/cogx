package org.cognitivesystems.comsys.data.testData;

import java.util.Vector;
import org.cognitivesystems.comsys.general.DialogueMoveUtils;
import org.cognitivesystems.comsys.autogen.ComsysEssentials.MoveType;


public class ProxyFactoriesTestData extends TestData {

	boolean logging = false;
	
	int unions ;
	
	public ProxyFactoriesTestData() {
		this.string = "";
	}
	
	public ProxyFactoriesTestData(String string, String unions) {
		this.string = string;
		this.unions = (new Integer(unions)).intValue();
	}
	
	public void setNbrUnions(String nbrUnions) {
		this.unions = (new Integer(nbrUnions)).intValue();
	}
	
	public int getNbrUnions() {
		return unions;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[PROXIES TEST DATA] " +  str);
	}
}
