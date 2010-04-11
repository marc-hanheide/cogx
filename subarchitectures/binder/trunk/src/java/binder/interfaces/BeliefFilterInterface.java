package binder.interfaces;

import java.util.Map;

import binder.autogen.beliefs.Belief;

public interface BeliefFilterInterface {
	public Map<Belief, Float> applyFilter(Map<Belief, Float> beliefs);
	
}
