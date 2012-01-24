package manipulation.core.share.exceptions;

/**
 * Exception for external memory failures
 * 
 * @author Torben Toeniges
 * 
 */
public class ExternalMemoryException extends Exception {

	private static final long serialVersionUID = -5842381106173711905L;

	/**
	 * exception constructor
	 */
	public ExternalMemoryException() {

	}

	/**
	 * exception constructor to define an error message
	 * 
	 * @param s
	 *            error message
	 */
	public ExternalMemoryException(String s) {
		super(s);
	}
}
