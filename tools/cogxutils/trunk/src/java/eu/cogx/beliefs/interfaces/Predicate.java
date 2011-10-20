package eu.cogx.beliefs.interfaces;

public interface Predicate extends BeliefStructure {
	public boolean get();
	public double getTrueProb();
	public void set(boolean val);
	public void set(boolean val, double prob);
}
