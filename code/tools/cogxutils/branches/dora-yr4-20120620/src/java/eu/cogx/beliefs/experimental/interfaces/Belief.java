package eu.cogx.beliefs.experimental.interfaces;

public interface Belief extends BeliefStructure {

	public FunctionOutcome<?> getFunction(String key);

	public String getId();

	public Predicate getPredicate(String key);

	public void put(String key, FunctionOutcome<?> func);

	public void put(String key);

	public void put(String key, Predicate val);

	public void remove(String key);

	public String setId();
	
	public Reference getReference();

}
