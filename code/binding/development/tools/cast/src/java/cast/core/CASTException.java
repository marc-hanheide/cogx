/**
 * 
 */
package cast.core;

/**
 * 
 * Basic CAST exception for any circumstances. Should be overridden for specific
 * exception types.
 * 
 * @author nah
 * 
 */
public class CASTException extends Exception {

	/**
	 * 
	 */
	private static final long serialVersionUID = 4722419583300841904L;

	/**
	 * 
	 */
	public CASTException() {
	}

	/**
	 * @param _message
	 */
	public CASTException(String _message) {
		super(_message);
	}

	/**
	 * @param _cause
	 */
	public CASTException(Throwable _cause) {
		super(_cause);
	}

	/**
	 * @param _message
	 * @param _cause
	 */
	public CASTException(String _message, Throwable _cause) {
		super(_message, _cause);
	}

}
