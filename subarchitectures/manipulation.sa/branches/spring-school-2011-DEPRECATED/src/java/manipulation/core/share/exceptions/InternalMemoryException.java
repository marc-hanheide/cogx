package manipulation.core.share.exceptions;

/**
 * Exception for internal memory failures
 * 
 * @author Torben Toeniges
 * 
 */
public class InternalMemoryException extends Exception {

	private static final long serialVersionUID = -7971843477952439571L;

	/**
	 * exception constructor
	 */
	public InternalMemoryException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public InternalMemoryException(String s) {
		super(s);
	}
}
