/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

public class EventID extends LongID {

    public EventID(EventID eid) {
	super(eid);
    }

    public EventID(long id) {
	super(id);
    }

}
