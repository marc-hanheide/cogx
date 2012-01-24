package de.dfki.lt.tr.dialogue.ref.potential;

public interface PotentialCombinator {

	/**
	 * Add a potential to the combination.
	 *
	 * @param p the potential
	 */
	public void addPotential(Potential p);

	/**
	 * Return the corresponding potential.
	 *
	 * @return the potential
	 */
	public Potential toPotential();

}
