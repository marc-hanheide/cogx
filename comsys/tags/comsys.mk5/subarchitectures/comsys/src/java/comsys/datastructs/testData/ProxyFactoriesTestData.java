package comsys.datastructs.testData;

import java.util.Vector;
import comsys.utils.DialogueMoveUtils;
import comsys.datastructs.comsysEssentials.MoveType;


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
