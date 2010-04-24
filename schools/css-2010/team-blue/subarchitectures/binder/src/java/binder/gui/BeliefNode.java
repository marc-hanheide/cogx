package binder.gui;

import beliefmodels.autogen.beliefs.Belief;

public class BeliefNode {
	
	public Belief belief;
	public String beliefid;
	
	public BeliefNode(Belief belief,String beliefid){
		this.belief = belief;
		this.beliefid = beliefid;
	}
	
	public BeliefNode(Belief belief){
		this.belief = belief;
		
	}
}
