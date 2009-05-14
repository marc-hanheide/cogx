/**
 * 
 */
package binding;

import cast.architecture.subarchitecture.SubarchitectureProcessException;


/**
 * @author nah
 *
 */
public class BindingException extends SubarchitectureProcessException {

    /**
     * 
     */
    private static final long serialVersionUID = -939202723715420453L;

    /**
     * 
     */
    public BindingException() {
    }

    /**
     * @param _message
     */
    public BindingException(String _message) {
        super(_message);
    }

    /**
     * @param _cause
     */
    public BindingException(Throwable _cause) {
        super(_cause);
    }

    /**
     * @param _message
     * @param _cause
     */
    public BindingException(String _message, Throwable _cause) {
        super(_message, _cause);
    }

}
