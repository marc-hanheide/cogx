package de.dfki.lt.tr.dialogue.intentions;

public interface Transcoder<Poor, Rich> {

	/**
	 * Return the appropriate decoded rich object or null if it is not
	 * possible.
	 * 
	 * @param obj  an encoded object
	 * @return  a decoded rich object or null if decoding didn't work
	 */
	public Rich tryDecode(Poor obj);

	/**
	 * Encode the given rich object into a poor representation.
	 * 
	 * @param obj  the rich object
	 * @return  encoded version of the rich object
	 */
	public Poor tryEncode(Rich obj);

}
