package org.cognitivesystems.reasoner.base;

/**
 * This interface defines what methods a Concept implementation should provide.
 * 
 * @author Hendrik Zender, zender@dfki.de
 * @version June 28, 2007
 */
public interface ReasonerConcept extends Comparable {

	/**
	 * @return the short name of the Concept
	 */
	public abstract String getName();
	
	/**
	 * @return the full name of the Concept
	 */
	public abstract String getFullName();
	
	/**
	 * @return the namespace of the Concept
	 */
	public abstract String getNamespace();

	/**
	 * @return the separator of the Concept
	 */
	public abstract String getSep();
	
}
