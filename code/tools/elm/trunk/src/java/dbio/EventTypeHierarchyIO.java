/** _
 *  part of the ELM system.
 *  @author Dennis Stachowicz
 */

package elm.dbio;

import elm.dbio.names.*;
import elm.event.*;
import elm.types.TypeHierarchy;

import java.sql.Connection;
import java.sql.SQLException;

public class EventTypeHierarchyIO 
    extends TypeHierarchyIO<EventType> {

     public EventTypeHierarchyIO(Connection c) throws SQLException {
	 
	 super(c, 
	       TableNames.event_type_hierarchy,
	       TableNames.eth_trans_closure,
	       ColumnNames.event_type,
	       ColumnNames.sub_type,
	       new EventTypeFactory(),
	       new EventTypeHierarchyFactory());
     }
}
