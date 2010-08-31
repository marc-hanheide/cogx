/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

public class StringID {

    private String id;

    public StringID(StringID eid) {
	id = eid.id;
    }

    public StringID(String id) {
	this.id = id;
    }

    public String toString() {
	return id;
    }
    public String getString() {
	return id;
    }
    
}
