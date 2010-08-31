/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import elm.types.TypeFactory;

public class EventTypeFactory implements TypeFactory<EventType> {

    public EventType getInstance(String typename) {
	return new EventType(typename);
    }
}
