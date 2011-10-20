package eu.cogx.beliefs.interfaces;

public interface FunctionValue extends BeliefStructure {
	public void set(FunctionValue val);
	public FunctionValue get();
	public boolean isNIL();
}
