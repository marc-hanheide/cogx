package eu.cogx.beliefs.experimental.interfaces;

public interface FunctionValue extends BeliefStructure {
	public void set(FunctionValue val);
	public FunctionValue get();
	public boolean isNIL();
}
