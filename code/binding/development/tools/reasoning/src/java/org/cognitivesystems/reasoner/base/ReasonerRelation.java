package org.cognitivesystems.reasoner.base;

/**
 * This interface defines what methods a Relation implementation should provide.
 * 
 * @author Hendrik Zender, zender@dfki.de
 * @version June 28, 2007
 */
public interface ReasonerRelation extends Comparable {

	/**
	 * @return the name of the Relation
	 */
	public abstract String getFullRelationName();
	
	/**
	 * @return the short name of the Relation
	 */
	public abstract String getRelationName();

	/**
	 * @return the namespace of the Relation
	 */
	public abstract String getRelationNamespace();

	/**
	 * @return the separator of the Relation
	 */
	public abstract String getRelationSep();

	/**
	 * @return the Relation's first argument (an Instance)
	 */
	public abstract ReasonerInstance getArg1();
	
	/**
	 * @return the Relation's second argument (an Instance)
	 */
	public abstract ReasonerInstance getArg2();
	
	
}
