/**
 * 
 */
package binding.common;

import cast.architecture.subarchitecture.SubarchitectureProcessException;


/**
 * @author henrikj
 *
 */
public class BindingComponentException extends SubarchitectureProcessException {

    /**
     * 
     */
    private static final long serialVersionUID = -2021169007968147496L;

    /**
     * 
     */
    public BindingComponentException() {
        super();
    }

    /**
     * @param _message
     */
    public BindingComponentException(String _message) {
        super(_message);
    }

    /**
     * @param _message
     * @param _cause
     */
    public BindingComponentException(String _message, Throwable _cause) {
        super(_message, _cause);
    }

    /**
     * @param _cause
     */
    public BindingComponentException(Throwable _cause) {
        super(_cause);
    }

}
