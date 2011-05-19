package manipulation.core.share.exceptions;

/**
 * Exception for failures in combination with an item
 * 
 * @author Torben Toeniges
 * 
 */
public class ItemException extends Exception {

	private static final long serialVersionUID = 2107305884517625510L;

	/**
	 * exception constructor
	 */
	public ItemException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public ItemException(String s) {
		super(s);
	}
}
