package comsys.datastructs.testData;

import java.util.Vector;
import comsys.utils.DialogueMoveUtils;
import comsys.datastructs.comsysEssentials.MoveType;


public class DiscRefTestData extends TestData {

	public class Reference {
		
		String refExpression;
		int refPosition;
		String referent;
		
		public Reference(String refExpression, String refPosition, String referent) {
			this.refExpression = refExpression;
			this.refPosition = (new Integer(refPosition)).intValue() - 1;
			if (!referent.equals("none")) {
				this.referent = referent;
			}
			else {
				this.referent = this.refExpression;
			}
		}
		
		public int getRefPosition() {
			return refPosition;
		}
		
		public String getRefExpression() {
			return refExpression;
		}
		
		public String getReferent() {
			return referent;
		}
	}
	
	boolean logging = false;
	
	Vector<String> discourse;
	Reference reference;
	
	
	public DiscRefTestData() {
		super();
	}
	
	public DiscRefTestData(Vector<String> discourse, Reference ref) {
		this.discourse = discourse;
		this.reference = ref;
	}
	
	
	public Vector<String> getDiscourse() {
		return discourse;
	}
	
	public void setReference(String refexpr, String refpos, String referent) {
		this.reference = new Reference(refexpr, refpos, referent);
	}
	
	public Reference getReference() {
		return reference;
	}
	public void setDiscourse(Vector<String> discourse) {
		this.discourse = discourse;
	}
	
	private void log(String str) {
		if (logging) System.out.println("[DiscRefData] " +  str);
	}
}
