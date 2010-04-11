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
	
	private static String OP_IMP = " => ";
	private static String OP_EQUI = " <=> ";
	
	public MLFormula(Float weight, String formula) {
		this.setWeight(weight);
		this.setFormula(formula);
	}

	public void setWeight(Float weight) {
		this.weight = weight;
	}

	public Float getWeight() {
		return weight;
	}

	public void setFormula(String formula) {
		this.formula = formula;
	}

	public String getFormula() {
		return formula;
	}
	
	public void addThisImpliesFormula(String formula) {
		this.formula = formula + OP_IMP + this.formula;
	}
	
	public void addThisEquivalence(String formula) {
		this.formula = formula + OP_EQUI + this.formula;
	}
	
	@Override
	public String toString() {
		return weight.toString() + " " + formula + "\n";
	}
}
