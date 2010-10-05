package de.dfki.lt.tr.dialmanagement.data.observations;

import java.util.Collection;
import java.util.HashMap;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;

public class Observation {

	private HashMap<ObservationContent,Float> alternatives ;
	
	public Observation () {
		alternatives = new HashMap<ObservationContent, Float>();
	}
	
	public Observation (ObservationContent content, float prob) {
		this();
		addAlternative(content,prob);
	}
	
	public Observation (dFormula formula, int type, float prob) {
		this (new ObservationContent (formula, type), prob);
	}
	
	public Observation (String utterance, float prob) {
		this (new ObservationContent(utterance), prob);
	}
	
	
	public Observation (HashMap<ObservationContent,Float> alternatives) {
		if (alternatives != null) {
			this.alternatives = alternatives;
		}
	}
	
	public void addAlternative (ObservationContent content, float prob) {
		alternatives.put(content, prob);
	}
	
	
	public void addAlternative (dFormula formula, int type, float prob) {
		alternatives.put(new ObservationContent (formula, type), prob);
	}
	
	public Collection<ObservationContent> getAlternatives () {
		return alternatives.keySet();
	}
	
	public float getProbability (ObservationContent content) {
		if (alternatives.containsKey(content)) {
			return alternatives.get(content);
		}
		else {
			return 0.0f;
		}
	}
	
	public String toString() {
		String result = "{";
		for (ObservationContent key : alternatives.keySet()) {
			result += "(" + key + ", "  + alternatives.get(key) + "), ";
		}
		return result.substring(0, result.length() -2) + "}";
	}
	
}
