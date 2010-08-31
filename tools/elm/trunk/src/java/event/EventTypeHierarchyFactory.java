/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.event;

import elm.types.TypeHierarchyFactory;
import elm.types.TypeHierarchy;

public class EventTypeHierarchyFactory implements TypeHierarchyFactory<EventType> {

    public TypeHierarchy<EventType> getInstance() {
	return new TypeHierarchy<EventType>();
    }
}
