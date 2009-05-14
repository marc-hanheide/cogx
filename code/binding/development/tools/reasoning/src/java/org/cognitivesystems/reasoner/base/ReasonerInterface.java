package org.cognitivesystems.reasoner.base;

import java.io.PrintStream;
import java.util.Set;


/**
 * A generic API for reasoners/inference engines.
 * The methods should be self-explaining...
 *
 * @author Hendrik Zender, zender@dfki.de
 * @version June 28, 2007
 */
public interface ReasonerInterface {
	
	public abstract Set<ReasonerInstance> getInstances(ReasonerConcept _con);
	
	public abstract Set<ReasonerInstance> getInstances(Set<ReasonerConcept> _cons);

	public abstract Set<ReasonerInstance> getInstancesByName(String _givenName);

	public abstract Set<ReasonerInstance> getInstancesByNumberTag(Integer _number);

	public abstract void addInstance(ReasonerInstance _ins, ReasonerConcept _con) throws ReasonerException;
	
	public abstract boolean deleteInstance(ReasonerInstance _instance);

	public abstract boolean deleteRelation(ReasonerRelation _relation);

	public abstract void assertRelation(ReasonerRelation _rel);
	
	public abstract Set<ReasonerConcept> getAllConcepts(ReasonerInstance _ins);

	public abstract boolean isInstanceOf(ReasonerInstance _instance, ReasonerConcept _concept);

	public abstract boolean isSuperConcept(ReasonerConcept _concept1, ReasonerConcept _concept2);

	public abstract boolean isSubConcept(ReasonerConcept _concept1, ReasonerConcept _concept2);

	public abstract boolean areConceptsEquivalent(ReasonerConcept _concept1, ReasonerConcept _concept2);
	
	public abstract boolean conceptExists(ReasonerConcept _con);
	
	public abstract boolean instanceExists(ReasonerInstance _ins);

	public abstract boolean relationExists(ReasonerRelation _rel);

	public abstract boolean areInstancesRelated(ReasonerInstance _ins1, ReasonerInstance _ins2, ReasonerRelation _rel);

	public abstract Set<ReasonerConcept> getDirectConcepts(ReasonerInstance _ins);
	
	public abstract Set<ReasonerConcept> getBasicLevelConcepts(ReasonerInstance _ins);

	public abstract Set<ReasonerConcept> getMostSpecificConcepts(ReasonerInstance _ins);
	
	public abstract Set<ReasonerConcept> getAllSubConcepts(ReasonerConcept _con);
	
	public abstract Set<ReasonerInstance> getPredecessors(ReasonerInstance _ins);
	
	public abstract Set<ReasonerInstance> getSuccessors(ReasonerInstance _ins);
	
	public abstract Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _ins); 
	
	public abstract Set<ReasonerInstance> getRelatedInstances(ReasonerInstance _ins, ReasonerRelation _rel);
	
	public abstract Set<ReasonerInstance> getInverseRelatedInstances(ReasonerInstance _ins, ReasonerRelation _rel);

	public abstract Set<ReasonerInstance> getImmediateRelatedInstances(ReasonerInstance _ins, ReasonerRelation _rel);
	
	public abstract Set<String> getNames(ReasonerInstance _ins);
	
	public abstract Set<Integer> getNumberTags(ReasonerInstance _ins);

	public abstract void addName(ReasonerInstance _ins, String _name);
	
	public abstract void addNumberTag(ReasonerInstance _ins, Integer _number);

	public abstract Set<Object> getPropertyValues(ReasonerInstance _ins, String _property);
	
	public abstract void addProperty(ReasonerInstance _ins, String _property, Object _value);

	public void setLogging(boolean _logging);
	
	public void setLogging(boolean _logging, PrintStream _outstream);
}
