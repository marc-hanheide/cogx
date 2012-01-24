package eu.cogx.beliefs.experimental.interfaces;

public interface FunctionOutcome<T extends FunctionValue> extends BeliefStructure,
		Iterable<T> {
	public void set(T value);
	public void set(T value, double prob);
	public double getProp(T value);
	public T mostLikely();
}
