
package comsys.processing.parse.examplegeneration;

import java.util.Arrays;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Vector;


public class Rule {

	String LHS;
	Vector<String> RHS;
	String semantics;
	Hashtable<String,Float> weights;

	public Rule() {
		RHS = new Vector<String>();
		weights = new Hashtable<String,Float>();
	}
	
	public Rule(String LHS, Vector<String> RHS) {
		this.LHS = LHS;
		this.RHS = RHS;
		weights = new Hashtable<String,Float>();
	}

	public Rule(String LHS, String RHS) {
		this.LHS = LHS;
		this.RHS = new Vector<String>(Arrays.asList(RHS.split(" ")));
	}
	
	public Rule(Vector<String> RHS) {
		this.RHS = RHS;
		weights = new Hashtable<String,Float>();
	}
	
	public void setSemantics(String semantics) {
		if (RHS.size() > 0) {
	/**	for (int i=1; i<10; i++) {
			String index = "@"+(new Integer(i)).toString();
		//	System.out.println(index);
			if (semantics.contains(index)) {
				semantics = semantics.replace(index, "@"+RHS.elementAt(i-1));
			}
		} */
		}
		this.semantics = semantics;
	}
	
	public void setLHS (String LHS) {
		this.LHS = LHS;
	}
	
	public void addSemantics(String sem) {
		semantics = sem;
	}
	
	public String getSemantics() {
		return semantics;
	}
	
	public void addToRHS(String str) {
		RHS.add(str);
	}
	
	public void addToRHS(Vector<String> strs) {
		RHS.addAll(strs);
	}

	public void addToRHS(Rule strs) {
		addToRHS(strs.getRHS());
	}
	
	public Vector<String> getRHS() {return RHS ; }

	public String getLHS() { return LHS; }

	public String toString() {
		String result = LHS + " ==> ";
		for (Enumeration<String> e = RHS.elements() ; e.hasMoreElements() ; ) {
			result += e.nextElement() + " ";
		}
		return result;
	}

	public String getRHSConstituent(int pos) {
		if (pos < RHS.size()) {
			return RHS.elementAt(pos);
		}
		else return null;
	}

	public void setWeight(String prevRule, float weight) {
		weights.put(prevRule, new Float(weight));
	}

}
