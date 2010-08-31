/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;


public class  WKTParseException 
    extends Exception {

    public WKTParseException(String message) {
	super(message);
    }

    public WKTParseException(Exception e) {
	super(e);
    }

}
