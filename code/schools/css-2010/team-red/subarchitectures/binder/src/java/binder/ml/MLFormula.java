package binder.ml;

/**
 * @author Carsten Ehrler
 * 
 * This class represents a single formula in Markov Logic, i.e.
 * a weight together with the first order logic formula 
 */
public class MLFormula {
	
	private Float weight;
	private String formula;
	private boolean sharp;
	
	private String OP_ADD;
	
	private static String OP_IMP = " => ";
	private static String OP_CONJ = " ^ ";
	
	public MLFormula(Float weight, String formula) {
		this.setWeight(weight);
		this.setFormula(formula);
		this.sharp = false;
		this.OP_ADD = OP_CONJ;
	}

	public void setWeight(Float weight) {
		this.weight = weight;
	}

	public Float getWeight() {
		return weight;
	}
	
	public void setSharp() {
		sharp = true;
	}
	
	public void setOpAddImp() {
		this.OP_ADD = OP_IMP;
	}
	
	public void setOpAddConj() {
		this.OP_ADD = OP_CONJ;
	}
	
	public void setFormula(String formula) {
		this.formula = formula;
	}

	public String getFormula() {
		return formula;
	}
	
	public void addThis(String formula) {
		this.formula = formula + OP_ADD + this.formula;
	}
	
	@Override
	public String toString() {
		if(sharp) {
			return formula + ".\n";
		}
		else {
		return weight.toString() + " " + formula + "\n";
		}
	}
}
